/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/eventgenerator/main_window.hpp"
#include "../include/eventgenerator/qnode.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace eventgenerator {

	using namespace Qt;


	/*****************************************************************************
	** Implementation [MainWindow]
	*****************************************************************************/

	MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
		: QMainWindow(parent)
		, qnode(argc,argv)
	{
		ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

		setWindowIcon(QIcon(":/images/icon.png"));
		ui.GeoPose_Tab->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

	    /*********************
	    ** Auto Start
	    **********************/

		double lat = 37.405109067;
		double lon = -121.918100758;
		float height = -10.136;
		float roll = 0;
		float pitch = 0;
		float heading = 0;
		float posRMS = 0;
		float ZUPT = 0;
		float headingRMS = 0;

		ui.lineEdit_lat->setText(QString::number(lat, 'g', 9));
		ui.lineEdit_lon->setText(QString::number(lon, 'g', 9));
		ui.lineEdit_height->setText(QString::number(height, 'f', 3));
		ui.lineEdit_roll->setText(QString::number(roll, 'f', 2));
		ui.lineEdit_pitch->setText(QString::number(pitch, 'f', 2));
		ui.lineEdit_heading->setText(QString::number(heading, 'f', 2));
		ui.lineEdit_posRMS->setText(QString::number(posRMS, 'f', 2));
		ui.lineEdit_ZUPT->setText(QString::number(ZUPT, 'f', 3));
		ui.lineEdit_headingRMS->setText(QString::number(headingRMS, 'f', 1));

		qnode.msg.Latitude = (double)(ui.lineEdit_lat->text()).toDouble();
		qnode.msg.Longitude = (double)(ui.lineEdit_lon->text()).toDouble();
		qnode.msg.EllipsoidalHeight = (float)(ui.lineEdit_height->text()).toFloat();
		qnode.msg.Roll = (short int)((ui.lineEdit_roll->text()).toFloat() * 100); 
		qnode.msg.Pitch = (short int)((ui.lineEdit_pitch->text()).toFloat() * 100); 
		qnode.msg.Heading = (short int)((ui.lineEdit_heading->text()).toFloat() * 100); 
		qnode.msg.PositionRMS = (unsigned short int)((ui.lineEdit_posRMS->text()).toFloat() * 100);
		qnode.msg.ZUPTRMS = (unsigned short int)((ui.lineEdit_ZUPT->text()).toFloat() * 1000);
		qnode.msg.HeadingRMS = (unsigned int)((ui.lineEdit_headingRMS->text()).toFloat() * 10);

		connect(ui.lineEdit_lat, SIGNAL(textChanged(const QString &)), this, SLOT(customSlot()));
		connect(ui.lineEdit_lon, SIGNAL(textChanged(const QString &)), this, SLOT(customSlot()));
		connect(ui.lineEdit_height, SIGNAL(textChanged(const QString &)), this, SLOT(customSlot()));
		connect(ui.lineEdit_posRMS, SIGNAL(textChanged(const QString &)), this, SLOT(customSlot()));
		connect(ui.lineEdit_ZUPT, SIGNAL(textChanged(const QString &)), this, SLOT(customSlot()));
		connect(ui.lineEdit_roll, SIGNAL(textChanged(const QString &)), this, SLOT(customSlot()));
	    connect(ui.lineEdit_pitch, SIGNAL(textChanged(const QString &)), this, SLOT(customSlot()));
	    connect(ui.lineEdit_heading, SIGNAL(textChanged(const QString &)), this, SLOT(customSlot()));
	    connect(ui.lineEdit_headingRMS, SIGNAL(textChanged(const QString &)), this, SLOT(customSlot()));

	    connect(ui.checkBox_GNSS, SIGNAL(stateChanged(int)), this, SLOT(customSlot()));
	    connect(ui.checkBox_roll, SIGNAL(stateChanged(int)), this, SLOT(customSlot()));
	   
	    ui.horizontalSlider_heading->setMinimum(-18000);
	    ui.horizontalSlider_heading->setMaximum(18000);


		connect(ui.horizontalSlider_heading, SIGNAL(valueChanged(int)), this, SLOT(updateTextOnSliderChange_heading(int)));

	    ui.horizontalSlider_headingRMS->setMinimum(0);
	    ui.horizontalSlider_headingRMS->setMaximum(255);
	    connect(ui.horizontalSlider_headingRMS, SIGNAL(valueChanged(int)), this, SLOT(updateTextOnSliderChange_headingRMS(int)));

	    ui.horizontalSlider_pitch->setMinimum(-9000);
	    ui.horizontalSlider_pitch->setMaximum(9000);
	    connect(ui.horizontalSlider_pitch, SIGNAL(valueChanged(int)), this, SLOT(updateTextOnSliderChange_pitch(int)));

	    ui.horizontalSlider_roll->setMinimum(-18000);
	    ui.horizontalSlider_roll->setMaximum(18000);
	    connect(ui.horizontalSlider_roll, SIGNAL(valueChanged(int)), this, SLOT(updateTextOnSliderChange_roll(int)));

	    ui.Slider_PositionRMS->setMinimum(0);
	    ui.Slider_PositionRMS->setMaximum(65535);
	    connect(ui.Slider_PositionRMS, SIGNAL(valueChanged(int)), this, SLOT(updateTextOnSliderChange_positionRMS(int)));

	    ui.horizontalSlider_ZUPT->setMinimum(0);
	    ui.horizontalSlider_ZUPT->setMaximum(65535);
	    connect(ui.horizontalSlider_ZUPT, SIGNAL(valueChanged(int)), this, SLOT(updateTextOnSliderChange_ZUPT(int)));

	    connect(ui.lineEdit_posRMS, SIGNAL(editingFinished()), this, SLOT(updateSlider_posRMS()));
	    connect(ui.lineEdit_ZUPT, SIGNAL(editingFinished()), this, SLOT(updateSlider_ZUPT()));
	    connect(ui.lineEdit_roll, SIGNAL(editingFinished()), this, SLOT(updateSlider_roll()));
	    connect(ui.lineEdit_pitch, SIGNAL(editingFinished()), this, SLOT(updateSlider_pitch()));
	    connect(ui.lineEdit_heading, SIGNAL(editingFinished()), this, SLOT(updateSlider_heading()));
	    connect(ui.lineEdit_headingRMS, SIGNAL(editingFinished()), this, SLOT(updateSlider_headingRMS()));

	    if (ui.pushButton_Activate->isChecked()) {
	    	on_pushButton_Activate_clicked(true);
	    }

	}

	MainWindow::~MainWindow() {}

	/*****************************************************************************
	** Implementation [Slots]
	*****************************************************************************/

	void MainWindow::customSlot() {
		qnode.msg.Latitude = (double)(ui.lineEdit_lat->text()).toDouble();
		qnode.msg.Longitude = (double)(ui.lineEdit_lon->text()).toDouble();
		qnode.msg.EllipsoidalHeight = (float)(ui.lineEdit_height->text()).toFloat();
		qnode.msg.Roll = (short int)((ui.lineEdit_roll->text()).toFloat() * 100); 
		qnode.msg.Pitch = (short int)((ui.lineEdit_pitch->text()).toFloat() * 100); 
		qnode.msg.Heading = (short int)((ui.lineEdit_heading->text()).toFloat() * 100); 
		qnode.msg.PositionRMS = (unsigned short int)((ui.lineEdit_posRMS->text()).toFloat() * 100);
		qnode.msg.ZUPTRMS = (unsigned short int)((ui.lineEdit_ZUPT->text()).toFloat() * 1000);
		qnode.msg.HeadingRMS = (unsigned int)((ui.lineEdit_headingRMS->text()).toFloat() * 10);

	    qnode.msg.Flags = 0;

	    if (ui.checkBox_roll->isChecked())
	    	qnode.msg.Flags |= 0x01;

	    if (ui.checkBox_GNSS->isChecked()) 
	    	qnode.msg.Flags |= 0x02;

	}

	void MainWindow::updateTextOnSliderChange_heading(int value) {
		float positionF = value / float (100);
	    ui.lineEdit_heading->setText( QString::number(positionF, 'f', 2));
	}

	void MainWindow::updateTextOnSliderChange_headingRMS(int value) {
		float positionF = value / float (10);
	    ui.lineEdit_headingRMS->setText( QString::number(positionF, 'f', 1) );
	}

	void MainWindow::updateTextOnSliderChange_pitch(int value) {
		float positionF = value / float (100);
	    ui.lineEdit_pitch->setText( QString::number(positionF, 'f', 2) );
	}

	void MainWindow::updateTextOnSliderChange_roll(int value) {
		float positionF = value / float (100);
	    ui.lineEdit_roll->setText( QString::number(positionF, 'f', 2) );
	}

	void MainWindow::updateTextOnSliderChange_positionRMS(int value) {
		float positionF = value / float (100);
	    ui.lineEdit_posRMS->setText( QString::number(positionF, 'f', 2) );
	}

	void MainWindow::updateTextOnSliderChange_ZUPT(int value) {
		float positionF = value / float (1000);
	    ui.lineEdit_ZUPT->setText( QString::number(positionF, 'f', 3) );
	}



	void MainWindow::updateSlider_posRMS() {
		float pos = (ui.lineEdit_posRMS->text()).toFloat() * 100;
		ui.Slider_PositionRMS->setValue(pos);
	}

	void MainWindow::updateSlider_ZUPT() {
		float pos = (ui.lineEdit_ZUPT->text()).toFloat() * 1000;
		ui.horizontalSlider_ZUPT->setValue(pos);
	}

	void MainWindow::updateSlider_roll() {
		float pos = (ui.lineEdit_roll->text()).toFloat() * 100;
		ui.horizontalSlider_roll->setValue(pos);
	}

	void MainWindow::updateSlider_pitch() {
		float pos = (ui.lineEdit_pitch->text()).toFloat() * 100;
		ui.horizontalSlider_pitch->setValue(pos);
	}

	void MainWindow::updateSlider_heading() {
		float pos = (ui.lineEdit_heading->text()).toFloat() * 100;
		ui.horizontalSlider_heading->setValue(pos);
	}

	void MainWindow::updateSlider_headingRMS() {
		float pos = (ui.lineEdit_headingRMS->text()).toFloat() * 10;
		ui.horizontalSlider_headingRMS->setValue(pos);
	}

	/*
	 * These triggers whenever the button is clicked, regardless of whether it
	 * is already checked or not.
	 */

	void MainWindow::on_pushButton_Activate_clicked(bool check ) {
		if ( !qnode.init() ) {
			printf("Could not start ROS!\n");
	 	} 
		if (ui.pushButton_Activate->text() != "Deactivate") {
			ui.pushButton_Activate->setText("Deactivate");

			ui.lineEdit_lat->setEnabled(false);
			ui.lineEdit_lon->setEnabled(false);
			ui.lineEdit_height->setEnabled(false);
			ui.lineEdit_posRMS->setEnabled(false);
			ui.lineEdit_heading->setEnabled(false);
			ui.lineEdit_headingRMS->setEnabled(false);
			ui.lineEdit_pitch->setEnabled(false);
			ui.lineEdit_roll->setEnabled(false);
			ui.Slider_PositionRMS->setEnabled(false);
			ui.horizontalSlider_heading->setEnabled(false);
			ui.horizontalSlider_headingRMS->setEnabled(false);
			ui.horizontalSlider_pitch->setEnabled(false);
			ui.horizontalSlider_roll->setEnabled(false);
			ui.horizontalSlider_ZUPT->setEnabled(false);
			ui.lineEdit_ZUPT->setEnabled(false);
			ui.checkBox_roll->setEnabled(false);
			ui.checkBox_GNSS->setEnabled(false);

		 } else {		 	
		 	qnode.activate = false;
		 	ui.pushButton_Activate->setText("Activate");

		 	ui.lineEdit_lat->setEnabled(true);
			ui.lineEdit_lon->setEnabled(true);
			ui.lineEdit_height->setEnabled(true);
			ui.lineEdit_posRMS->setEnabled(true);
			ui.lineEdit_heading->setEnabled(true);
			ui.lineEdit_headingRMS->setEnabled(true);
			ui.lineEdit_pitch->setEnabled(true);
			ui.lineEdit_roll->setEnabled(true);
			ui.Slider_PositionRMS->setEnabled(true);
			ui.horizontalSlider_heading->setEnabled(true);
			ui.horizontalSlider_headingRMS->setEnabled(true);
			ui.horizontalSlider_pitch->setEnabled(true);
			ui.horizontalSlider_roll->setEnabled(true);
			ui.horizontalSlider_ZUPT->setEnabled(true);
			ui.lineEdit_ZUPT->setEnabled(true);
			ui.checkBox_roll->setEnabled(true);
			ui.checkBox_GNSS->setEnabled(true);
		 }
		
	}

	void MainWindow::closeEvent(QCloseEvent *event) {	
		QMainWindow::closeEvent(event);
	}
}  // namespace eventgenerator

