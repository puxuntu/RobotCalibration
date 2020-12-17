#pragma once

#include <QWidget>
#include "ui_ToolCalibration.h"
#include "NDI.h"
#include <QTimer>
#include <Eigen/Dense>
#include "UR_interface.h"
#include <vector>
#include "Calibration.h"
#include <fstream>

typedef Eigen::Matrix4d Matrix4d;

class ToolCalibration : public QWidget
{
	Q_OBJECT

public:
	ToolCalibration(UR_interface* robot, NDI* ndi, Matrix4d matrix, int rRef, int cRef, QWidget *parent = Q_NULLPTR);
	~ToolCalibration();
	bool isCalibrationFinished();

private:
	Ui::ToolCalibration ui;
	UR_interface* m_robot;
	NDI* m_device;
	int robotRef;
	int caliRef;
	bool isCalibrated;
	Matrix4d matrixBaseRref;
	//��Ϊ�ڵ���궨��rom�ļ���ʱ��������ƫ�ƣ����Ծ���matrixRrefCalibrator
	vector<Matrix4d> matrixRrefTool;

	QTimer* m_timer;

	Matrix4d getCalibrationMatrix();

private slots:
	void OnCalibration();
	void OnLoad();
	void OnCheckTimeout();
};
