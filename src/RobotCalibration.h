#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_RobotCalibration.h"
#include <QMessageBox>
#include "NDI.h"
#include "UR_interface.h"
#include <string>
#include <math.h>
#include <QTimer>
#include "Calibration.h"
#include "ToolCalibration.h"
enum state {
	start, stop
};

class RobotCalibration : public QMainWindow
{
    Q_OBJECT

public:
    RobotCalibration(QWidget *parent = Q_NULLPTR);
	~RobotCalibration();

private:
    Ui::RobotCalibrationClass ui;
	NDI* m_device;
	UR_interface* m_robot;
	QTimer* m_timer;
	Calibration* m_robotCali;
	ToolCalibration* m_toolCali;
	state m_state;
	string ip;

	int caliRef;
	int robotRef;
	int probe;
	int calibrator;

	Matrix4d caliMatrix;//Transform base to robot reference,µ¥Î»ÊÇm

	Matrix4d mat2Matrix4d(const double mat[4][4]);
	void Matrix4d2mat(const Matrix4d matrix, double mat[4][4]);
	void printMat(const double mat[4][4], string s);
	void initConnection();

private slots:
	void OnLoadRef();
	void OnStart();
	void OnRobotCalibration();
	void OnTracking();
	void OnCheckTimeOut();
	void OnToolCalibration();
};
