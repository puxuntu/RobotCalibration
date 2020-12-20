#include "ToolCalibration.h"

ToolCalibration::ToolCalibration(UR_interface* robot, NDI* ndi, Matrix4d matrix, int rRef, int cRef, QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	m_robot = robot;
	m_device = ndi;
	robotRef = rRef;
	caliRef = cRef;
	matrixBaseRref = matrix;
	isCalibrated = false;
	m_timer = new QTimer(this);

	connect(ui.startButton, SIGNAL(clicked()), this, SLOT(OnCalibration()));
	connect(ui.loadButton, SIGNAL(clicked()), this, SLOT(OnLoad()));
	ui.progressBar->setMinimum(0);
	ui.progressBar->setMaximum(100);
	ui.progressBar->setValue(0);
}

ToolCalibration::~ToolCalibration()
{
}

bool ToolCalibration::isCalibrationFinished()
{
	return isCalibrated;
}

Matrix4d ToolCalibration::getCalibrationMatrix()
{
	//对matrixRrefTool进行平均
	double posSum[7] = { 0 };
	for (int i = 0; i < matrixRrefTool.size(); i++)
	{
		double mat[4][4], pos[6];
		Calibration::Matrix4d2mat(matrixRrefTool[i], mat);
		UR_interface::matrix_2_UR6params(mat, pos);

		posSum[0] = +pos[0] / 1000;
		posSum[1] = +pos[1] / 1000;
		posSum[2] = +pos[2] / 1000;

		double angle = sqrt(pos[3] * pos[3] + pos[4] * pos[4] + pos[5] * pos[5]);
		//这里加的是轴的平方，保证平均之后仍然是单位向量
		posSum[3] = +pow(pos[3] / angle, 2);
		posSum[4] = +pow(pos[4] / angle, 2);
		posSum[5] = +pow(pos[5] / angle, 2);
		posSum[6] = +angle;
	}
	for (int i = 0; i < 7; i++)
	{
		posSum[i] /= matrixRrefTool.size();
	}
	double pos[6], mat[4][4];
	for (int i = 0; i < 3; i++)
	{
		pos[i] = posSum[i];
		pos[i + 3] = sqrt(posSum[i + 3])*pos[6];
	}
	UR_interface::UR6params_2_matrix(pos, mat);
	Matrix4d aveMatrixRrefTool = Calibration::mat2Matrix4d(mat);

	double tcpPos[6] = { 0 };
	m_robot->SetTCPPos(tcpPos);
	m_robot->GetTCPPos(pos);
	UR_interface::UR6params_2_matrix(pos, mat);
	Matrix4d matrixTcpBase = Calibration::mat2Matrix4d(mat);

	Matrix4d matrixTcpTool = aveMatrixRrefTool * matrixBaseRref * matrixTcpBase;
	return matrixTcpTool.inverse();
}

void ToolCalibration::OnCalibration()
{
	connect(m_timer, SIGNAL(timeout()), this, SLOT(OnCheckTimeout()));
	m_timer->start(100);
}

void ToolCalibration::OnCheckTimeout()
{
	static int num = 0;
	Matrix4d matrixRrefCali;
	if (m_device->getToolTransformationMatrix(robotRef, caliRef, matrixRrefCali))
	{
		matrixRrefTool.push_back(matrixRrefCali);
		num++;
		ui.progressBar->setValue(num);
		if (num == 100)
		{
			m_timer->stop();
			disconnect(m_timer, SIGNAL(timeout()), this, SLOT(OnCheckTimeout()));
			Matrix4d matrixToolTcp = getCalibrationMatrix();
			cout << "tool cali matrix" << endl;
			cout << matrixToolTcp << endl;

			double mat[4][4], pos[6];
			Calibration::Matrix4d2mat(matrixToolTcp, mat);
			UR_interface::matrix_2_UR6params(mat, pos);
			m_robot->SetTCPPos(pos);
			//保存标定结果
			ofstream file("..\\data\\toolCaliData.txt");
			if (!file.is_open())
			{
				cout << "can not open tool cali file" << endl;
				return;
			}
			file << matrixToolTcp << endl << endl;
			file.close();

			this->close();
		}
	}
	else
	{
		return;
	}
}

void ToolCalibration::OnLoad()
{
	Matrix4d matrixToolTcp;
	ifstream m_caliFile("..\\data\\toolCaliData.txt");
	if (!m_caliFile.is_open())
	{
		cout << "can not open cali file" << endl;
		return;
	}
	for (int j = 0; j < 4; ++j)
	{
		for (int k = 0; k < 4; ++k)
		{
			m_caliFile >> matrixToolTcp(j, k);
		}
	}
	cout << "tool cali matrix" << endl;
	cout << matrixToolTcp << endl;

	double mat[4][4], pos[6];
	Calibration::Matrix4d2mat(matrixToolTcp, mat);
	UR_interface::matrix_2_UR6params(mat, pos);
	m_robot->SetTCPPos(pos);
	this->close();
}