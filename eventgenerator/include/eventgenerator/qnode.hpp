/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef eventgenerator_QNODE_HPP_
#define eventgenerator_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "polyx_nodea/StaticGeoPoseEvent.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace eventgenerator {

/*****************************************************************************
** Class
*****************************************************************************/

	class QNode : public QThread {
	    Q_OBJECT
	public:
		QNode(int argc, char** argv );
		virtual ~QNode();
		bool init();
		bool init(const std::string &master_url, const std::string &host_url);
		void run();

		/*********************
		** Logging
		**********************/
		enum LogLevel {
			Debug,
			Info,
			Warn,
			Error,
			Fatal
		};

	public:
		int init_argc;
		char** init_argv;

		polyx_nodea::StaticGeoPoseEvent msg;
		ros::Publisher staticGeoPose_pub;

		bool activate;
	};

}  // namespace eventgenerator

#endif /* eventgenerator_QNODE_HPP_ */
