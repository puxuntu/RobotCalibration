#pragma once

#include <QWidget>
#include "ui_Calibration.h"
#include <fstream>
#include <string>
#include "NDI.h"
#include "UrAPI/UR_interface.h"
#include <Eigen/Dense>
#define PI 3.1415926

typedef Eigen::Matrix<double, 3, 9> Matrix39d;
typedef Eigen::Matrix<double, 18, 3> Matrix183d;
typedef Eigen::Matrix<double, 18, 1> Vector18d;

class Calibration : public QWidget
{
	Q_OBJECT

public:
	Calibration(UR_interface* robot, NDI* ndi, int rRef, int cRef, QWidget *parent = Q_NULLPTR);
	~Calibration();
	Matrix4d getMatrix();
	bool isCalibrationFinished();//是否完成机器人标定

	static Matrix4d mat2Matrix4d(const double mat[4][4]);
	static void Matrix4d2mat(const Matrix4d matrix, double mat[4][4]);
	static void printMat(const double mat[4][4], string s);

private:
	Ui::Calibration ui;
	UR_interface* m_robot;
	NDI* m_device;
	int robotRef;
	int caliRef;

	vector<Matrix4d> matrixEndBase;
	vector<Matrix4d> matrixRobotCali;
	Matrix4d caliMatrix;//Transform base to robot reference,单位是m

	bool isCalibrated;

	bool isReach(const double pos[6]);
	Vector3d getKVector(Matrix4d matrix);
	//num是第几组
	void getSeriesKVector(int num, Vector3d& KA1, Vector3d& KA2, Vector3d& KB1, Vector3d& KB2,
		Matrix4d& MA1, Matrix4d& MA2, Matrix4d& MB1, Matrix4d& MB2);
	Vector3d getT(Matrix4d A, Matrix4d B, Matrix3d R);
	Matrix4d calibrationMatrix();

private slots:
	void OnCalibration();
	void OnCollection();
	void OnAuto();
	void OnLoadData();
};
