/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef eventgenerator_MAIN_WINDOW_H
#define eventgenerator_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace eventgenerator {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
	class MainWindow : public QMainWindow {
		Q_OBJECT //declares our class as a QObject

		public:
			MainWindow(int argc, char** argv, QWidget *parent = 0);
			~MainWindow();

			void closeEvent(QCloseEvent *event); // Overloaded function

		public Q_SLOTS:
			void on_pushButton_Activate_clicked(bool check );
			void customSlot();
			void updateTextOnSliderChange_heading(int value);
			void updateTextOnSliderChange_headingRMS(int value);
			void updateTextOnSliderChange_pitch(int value);
			void updateTextOnSliderChange_roll(int value);
			void updateTextOnSliderChange_positionRMS(int value);
			void updateTextOnSliderChange_ZUPT(int value);

			void updateSlider_posRMS();
			void updateSlider_ZUPT();
			void updateSlider_roll();
			void updateSlider_pitch();
			void updateSlider_heading();
			void updateSlider_headingRMS();

		public:
			Ui::MainWindowDesign ui;
			QNode qnode;
	};

}  // namespace eventgenerator

#endif // eventgenerator_MAIN_WINDOW_H
