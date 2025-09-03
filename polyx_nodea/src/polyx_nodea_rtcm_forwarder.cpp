/*
 * Copyright (C) 2025, Polynesian Exploration Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <future>
#include <unordered_map>

#include "ros/ros.h"
#include "polyx_nodea/BinaryData.h"

using std::placeholders::_1;

uint64_t getAddrID(const sockaddr_in &addr)
{
    return (static_cast<uint64_t>(addr.sin_addr.s_addr) << 32 | addr.sin_port);
}

class PolyxRtcmForwarder
{
private:
    // flags
    bool is_valid_;
    bool is_tcp_;
    std::atomic<bool> shall_stop_bg_;
    // server
    int server_fd_ = -1;
    sockaddr_in serv_addr_;
    std::future<void> bg_future_;
    // subscriber
    ros::Subscriber subscriber_;
    // mutex
    std::mutex client_mutex_;

    // TCP related
    std::vector<int> tcp_client_fd_list_;

    // UDP related
    struct UDP_ClientInfo
    {
        sockaddr_in address;
        std::chrono::steady_clock::time_point last_received_time;
        std::vector<uint8_t> buffer;
        int data_offset;
    };
    std::unordered_map<uint64_t, UDP_ClientInfo> udp_clients_;

    bool initServerSocket()
    {
        // new socket
        server_fd_ = socket(AF_INET, is_tcp_ ? SOCK_STREAM : SOCK_DGRAM, 0);
        if (server_fd_ == -1)
        {
            ROS_ERROR("Cannot open socket.");
            return false;
        }

        // set options
        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // bind
        if (bind(server_fd_, reinterpret_cast<sockaddr *>(&serv_addr_), sizeof(serv_addr_)) != 0)
        {
            ROS_ERROR("Socket bind failed.");
            close(server_fd_);
            return false;
        }

        // listen
        if (is_tcp_ && listen(server_fd_, 1000) != 0)
        {
            ROS_ERROR("Socket listen failed");
            close(server_fd_);
            return false;
        }

        ROS_INFO("Server set.");

        return true;
    }

    void TCP_acceptConnections()
    {
        while (!shall_stop_bg_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // accept
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd_, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
            if (client_fd < 0)
            {
                ROS_ERROR("Cannot accept connection request.");
                continue;
            }
            ROS_INFO("New client connected: %s:%d",
                        inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

            // add to list
            std::lock_guard<std::mutex> lock(client_mutex_);
            tcp_client_fd_list_.push_back(client_fd);
        }
    }

    void UDP_receiveData()
    {
        char buffer[1024];
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        while (!shall_stop_bg_)
        {
            ssize_t n = recvfrom(server_fd_, buffer, sizeof(buffer), 0,
                                 reinterpret_cast<sockaddr *>(&client_addr), &client_len);

            if (n < 0)
            {
                ROS_ERROR("UDP server [recvfrom] error.");
                continue;
            }

            // prep
            auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(client_mutex_);

            // check if client is new or existing
            uint64_t key = getAddrID(client_addr);
            auto it = udp_clients_.find(key);
            if (it == udp_clients_.end())
            {
                // new client
                UDP_ClientInfo new_client;
                new_client.address = client_addr;
                new_client.last_received_time = now;
                new_client.data_offset = 0;
                udp_clients_[key] = new_client;

                ROS_INFO("New client: %s:%d.",
                            inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
            }
            else
            {
                // existing client, update
                it->second.last_received_time = now;
            }
        }
    }

    void TCP_callback(const polyx_nodea::BinaryData &msg)
    {
        const uint8_t *data_ptr = msg.data.data();
        size_t data_size = msg.data.size();
        ROS_INFO("Receiving %zu bytes from topic.", data_size);

        // send
        ssize_t r = 0;
        for (auto it = tcp_client_fd_list_.begin(); it != tcp_client_fd_list_.end();)
        {
            r = send(*it, data_ptr, data_size, MSG_NOSIGNAL);

            if (r <= 0)
            {
                ROS_ERROR("Failed to send data to client socket %d.", *it);
                std::lock_guard<std::mutex> lock(client_mutex_);
                it = tcp_client_fd_list_.erase(it);
                ROS_INFO("Socket closed.");
            }
            else
            {
                ROS_INFO("Sent %zu bytes to socket %d.", data_size, *it);
                it++;
            }
        }
    }

    void UDP_callback(const polyx_nodea::BinaryData &msg)
    {
        const std::vector<uint8_t> &data = msg.data;
        ROS_INFO("Receiving %zu bytes from topic.", data.size());
        std::lock_guard<std::mutex> lock(client_mutex_);

        std::vector<uint64_t> keys_to_be_removed;
        for (auto &[key, client] : udp_clients_)
        {
            // check client connectivity
            auto now = std::chrono::steady_clock::now();
            if (client.last_received_time + std::chrono::seconds(10) < now)
            {
                keys_to_be_removed.push_back(key);
                continue;
            }

            // append buffer
            client.buffer.insert(client.buffer.end(), data.begin(), data.end());

            // send 512*2*N
            while (client.buffer.size() - client.data_offset >= 1024)
            {
                for (int i = 0; i < 2; i++)
                {
                    if (!sendUDP_data(client, 512))
                    {
                        keys_to_be_removed.push_back(key);
                        continue;
                    }
                }
            }
            // send rest
            if (client.buffer.size() - client.data_offset >= 512)
            {
                if (!sendUDP_data(client, client.buffer.size() - client.data_offset))
                {
                    keys_to_be_removed.push_back(key);
                    continue;
                }
            }

            // update
            client.buffer = {client.buffer.begin() + client.data_offset, client.buffer.end()};
            client.data_offset = 0;
        }

        for (auto e : keys_to_be_removed)
        {
            const auto client = udp_clients_[e];
            udp_clients_.erase(e);
            ROS_INFO("Client %s:%d closed.",
                        inet_ntoa(client.address.sin_addr), ntohs(client.address.sin_port));
        }
    }

    bool sendUDP_data(UDP_ClientInfo &client, int data_len)
    {
        // sendto
        ssize_t r = sendto(server_fd_, client.buffer.data() + client.data_offset, data_len, MSG_NOSIGNAL,
                           reinterpret_cast<const struct sockaddr *>(&client.address), sizeof(client.address));

        // check return
        if (r < 0)
        {
            ROS_ERROR("Failed to send data to client %s:%d",
                         inet_ntoa(client.address.sin_addr), ntohs(client.address.sin_port));
            return false;
        }
        else if (static_cast<int>(r) != data_len)
        {
            ROS_WARN("Partial UDP data (%zd/%d bytes) sent to %s:%d.",
                        r, data_len, inet_ntoa(client.address.sin_addr), ntohs(client.address.sin_port));
        }
        else
        {
            ROS_INFO("Sent %d bytes to client %s:%d.",
                        data_len, inet_ntoa(client.address.sin_addr), ntohs(client.address.sin_port));
        }

        // update
        client.data_offset += data_len;
        return true;
    }

public:
    PolyxRtcmForwarder(
        bool is_tcp, sockaddr_in serv_addr, const std::string& rtcm_topic, ros::NodeHandle& nh)
        : is_valid_(false), is_tcp_(is_tcp), shall_stop_bg_(false), serv_addr_(serv_addr)
    {
        // initialize server socket
        if (!initServerSocket())
        {
            return;
        }

        // create ROS subscriber
        subscriber_ = nh.subscribe(rtcm_topic, 10, is_tcp_ ?
            &PolyxRtcmForwarder::TCP_callback : &PolyxRtcmForwarder::UDP_callback, this);

        // run background thread
        bg_future_ = std::async(std::launch::async, is_tcp_ ? &PolyxRtcmForwarder::TCP_acceptConnections : &PolyxRtcmForwarder::UDP_receiveData, this);

        is_valid_ = true;
    }

    ~PolyxRtcmForwarder()
    {
        shall_stop_bg_ = false;
        if (bg_future_.valid())
        {
            bg_future_.wait();
        }

        if (is_valid_)
        {
            close(server_fd_);
        }
    }

    bool isValid() { return is_valid_; }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "polyx_node_rtcm_forwarder");
    ros::NodeHandle nh;

    // get parameters
    std::string local_ip;
    if (nh.getParam("/local_ip", local_ip))
    {
        ROS_INFO("Server IP address: %s.", local_ip.c_str());
    }
    else
    {
        ROS_ERROR("Cannot retrieve server IP address.");
        return 1;
    }
    //
    bool is_tcp = true;
    nh.getParam("/use_tcp", is_tcp);
    ROS_INFO("Using %s.", is_tcp ? "TCP" : "UDP");
    //
    std::string local_port;
    if (nh.getParam("/local_port", local_port))
    {
        ROS_INFO("Server port: %s.", local_port.c_str());
    }
    else
    {
        ROS_ERROR("Cannot retrieve server port.");
        return 1;
    }
    //
    std::string rtcm_topic;
    if (nh.getParam("/rtcm_data_topic", rtcm_topic))
    {
        ROS_INFO("RTCM data topic: %s.", rtcm_topic.c_str());
    }
    else
    {
        ROS_ERROR("Cannot retrieve RTCM data topic.");
        return 1;
    }

    // set server addresss
    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(local_ip.c_str());
    uint32_t port = static_cast<uint32_t>(std::stoul(local_port));
    serv_addr.sin_port = htons(port);

    // new
    auto rtcm_forwarder = std::make_unique<PolyxRtcmForwarder>(
        is_tcp, serv_addr, rtcm_topic, nh);

    // run
    ros::spin();

    return 0;
}
