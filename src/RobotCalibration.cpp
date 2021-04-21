#include "RobotCalibration.h"

RobotCalibration::RobotCalibration(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	m_robot = new UR_interface();
	m_device = new NDI();
	m_timer = new QTimer(this);
	m_robotCali = nullptr;
	m_toolCali = nullptr;
	m_state = stop;
	m_device->initDevice();
	initConnection();
	ip = "169.254.174.11";
}

RobotCalibration::~RobotCalibration()
{
	m_timer->stop();
	m_robot->Stopj();
}

void RobotCalibration::initConnection()
{
	connect(ui.addRefButton, SIGNAL(clicked()), this, SLOT(OnLoadRef()));
	connect(ui.startButton, SIGNAL(clicked()), this, SLOT(OnStart()));
	connect(ui.robotCalibrationButton, SIGNAL(clicked()), this, SLOT(OnRobotCalibration()));
	connect(ui.trackButton, SIGNAL(clicked()), this, SLOT(OnTracking()));
	connect(ui.toolCalibrationButton, SIGNAL(clicked()), this, SLOT(OnToolCalibration()));
}

Matrix4d RobotCalibration::mat2Matrix4d(const double mat[4][4])
{
	Matrix4d matrix;
	matrix << mat[0][0], mat[0][1], mat[0][2], mat[0][3],
		mat[1][0], mat[1][1], mat[1][2], mat[1][3],
		mat[2][0], mat[2][1], mat[2][2], mat[2][3],
		mat[3][0], mat[3][1], mat[3][2], mat[3][3];
	return matrix;
}

void RobotCalibration::Matrix4d2mat(const Matrix4d matrix, double mat[4][4])
{
	mat[0][0] = matrix(0, 0);
	mat[0][1] = matrix(0, 1);
	mat[0][2] = matrix(0, 2);
	mat[0][3] = matrix(0, 3);
	mat[1][0] = matrix(1, 0);
	mat[1][1] = matrix(1, 1);
	mat[1][2] = matrix(1, 2);
	mat[1][3] = matrix(1, 3);
	mat[2][0] = matrix(2, 0);
	mat[2][1] = matrix(2, 1);
	mat[2][2] = matrix(2, 2);
	mat[2][3] = matrix(2, 3);
	mat[3][0] = matrix(3, 0);
	mat[3][1] = matrix(3, 1);
	mat[3][2] = matrix(3, 2);
	mat[3][3] = matrix(3, 3);
}

void RobotCalibration::printMat(const double mat[4][4], string s)
{
	cout << s << endl;
	cout << mat[0][0] << "," << mat[0][1] << "," << mat[0][2] << "," << mat[0][3] << endl;
	cout << mat[1][0] << "," << mat[1][1] << "," << mat[1][2] << "," << mat[1][3] << endl;
	cout << mat[2][0] << "," << mat[2][1] << "," << mat[2][2] << "," << mat[2][3] << endl;
	cout << mat[3][0] << "," << mat[3][1] << "," << mat[3][2] << "," << mat[3][3] << endl;
}

void RobotCalibration::OnLoadRef()
{
	if (m_device->loadTool("..\\data\\calibration.rom", caliRef))
	{
		cout << "Add calibration reference!" << endl;
	}
	if (m_device->loadTool("..\\data\\robot.rom", robotRef))
	{
		cout << "Add robot refrence!" << endl;
	}
	if (m_device->loadTool("..\\data\\probe.rom", probe))
	{
		cout << "Add probe refrence!" << endl;
	}
	if (m_device->loadTool("..\\data\\calibrator.rom", calibrator))
	{
		cout << "Add calibrator refrence!" << endl;
	}

	m_device->initTools();

	//ÅäÖÃ±ê¶¨¿éµÄÆ«ÒÆ¾ØÕó
	Matrix4d calibratorDevMatrix;
	calibratorDevMatrix << 1, 0, 0, -19.5, 0, 1, 0, 0, 0, 0, 1, -13.64;
	m_device->setDeviateMatrix(calibrator, calibratorDevMatrix);
}

void RobotCalibration::OnStart()
{
	m_device->startTracking();
	cout << "Start tracking!" << endl;

	m_robot->connect_robot(ip);

	if (m_robot->isConnected())
	{
		double pos[6] = { 0 };
		m_robot->SetTCPPos(pos);
		Beep(1000, 500);
		cout << "robot is connected!" << endl;
	}
	else
	{
		QMessageBox::warning(0, "robot connection", "robot is not connected!");
		return;
	}

	connect(m_timer, SIGNAL(timeout()), this, SLOT(OnCheckTimeOut()));
	m_timer->start(100);
}

void RobotCalibration::OnRobotCalibration()
{
	if (m_robotCali)
	{
		return;
	}
	double pos[6] = { 0 };
	m_robot->GetTCPPos(pos);
	for (int i = 0; i < 6; i++)
	{
		cout << pos[i] << "  ";
	}
	cout << endl;

	m_robotCali = new Calibration(m_robot, m_device, robotRef, caliRef);
	m_robotCali->show();
}

void RobotCalibration::OnTracking()
{
	if (m_robotCali!=nullptr && m_robotCali->isCalibrationFinished())
	{
		caliMatrix = m_robotCali->getMatrix();
		cout << caliMatrix << endl;
		
		if (m_state == start)
		{
			m_state = stop;
			m_robot->Stopl();
		}
		else if (m_state == stop)
		{
			m_state = start;
		}
	}
	else
	{
		return;
	}
}

void RobotCalibration::OnCheckTimeOut()
{
	if (m_device->isToolMissing(robotRef)) ui.robotButton->setStyleSheet("background-color: rgb(255,0,0)");
	else ui.robotButton->setStyleSheet("background-color: rgb(0,255,0)");
	if (m_device->isToolMissing(caliRef)) ui.caliButton->setStyleSheet("background-color: rgb(255,0,0)");
	else ui.caliButton->setStyleSheet("background-color: rgb(0,255,0)");
	if (m_device->isToolMissing(probe)) ui.probeButton->setStyleSheet("background-color: rgb(255,0,0)");
	else ui.probeButton->setStyleSheet("background-color: rgb(0,255,0)");
	if (m_device->isToolMissing(calibrator)) ui.calibratorButton->setStyleSheet("background-color: rgb(255,0,0)");
	else ui.probeButton->setStyleSheet("background-color: rgb(0,255,0)");

	if (m_state == start)
	{
		Matrix4d matrixProbeRobot;
		if (!m_device->getToolTransformationMatrix(probe, robotRef, matrixProbeRobot))
		{
			m_robot->Stopl();
			return;
		}
		matrixProbeRobot(0, 3) /= 1000;
		matrixProbeRobot(1, 3) /= 1000;
		matrixProbeRobot(2, 3) /= 1000;
		Matrix4d matrixEndProbe;
		matrixEndProbe << 0, 0, 1, -0.15, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

		Matrix4d matrixEndBase = caliMatrix.inverse()*matrixProbeRobot*matrixEndProbe;
		double pos[6], mat[4][4];
		Matrix4d2mat(matrixEndBase, mat);
		m_robot->matrix_2_UR6params(mat, pos);

		m_robot->Movel_pose(pos);
	}
	else
	{
		return;
	}
}

void RobotCalibration::OnToolCalibration()
{
	if (m_robotCali == nullptr || !m_robotCali->isCalibrationFinished())	return;
	if (m_toolCali) return;

	caliMatrix = m_robotCali->getMatrix();
	m_toolCali = new ToolCalibration(m_robot, m_device, caliMatrix, robotRef, calibrator);
	m_toolCali->show();
}

