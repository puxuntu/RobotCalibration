#include "Calibration.h"


Calibration::Calibration(UR_interface* robot, NDI* ndi, int rRef, int cRef, QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	m_robot = robot;
	m_device = ndi;
	robotRef = rRef;
	caliRef = cRef;
	isCalibrated = false;
	connect(ui.calibrationButton, SIGNAL(clicked()), this, SLOT(OnCalibration()));
	connect(ui.collectButton, SIGNAL(clicked()), this, SLOT(OnCollection()));
	connect(ui.autoButton, SIGNAL(clicked()), this, SLOT(OnAuto()));
	connect(ui.loadButton, SIGNAL(clicked()), this, SLOT(OnLoadData()));
}

Calibration::~Calibration()
{

}

Matrix4d Calibration::getMatrix()
{
	return caliMatrix;
}

Matrix4d Calibration::mat2Matrix4d(const double mat[4][4])
{
	Matrix4d matrix;
	matrix << mat[0][0], mat[0][1], mat[0][2], mat[0][3],
		mat[1][0], mat[1][1], mat[1][2], mat[1][3],
		mat[2][0], mat[2][1], mat[2][2], mat[2][3],
		mat[3][0], mat[3][1], mat[3][2], mat[3][3];
	return matrix;
}

void Calibration::Matrix4d2mat(const Matrix4d matrix, double mat[4][4])
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

void Calibration::printMat(const double mat[4][4], string s)
{
	cout << s << endl;
	cout << mat[0][0] << "," << mat[0][1] << "," << mat[0][2] << "," << mat[0][3] << endl;
	cout << mat[1][0] << "," << mat[1][1] << "," << mat[1][2] << "," << mat[1][3] << endl;
	cout << mat[2][0] << "," << mat[2][1] << "," << mat[2][2] << "," << mat[2][3] << endl;
	cout << mat[3][0] << "," << mat[3][1] << "," << mat[3][2] << "," << mat[3][3] << endl;
}

bool Calibration::isReach(const double pos[6])
{
	double realPos[6];
	m_robot->GetTCPPos(realPos);
	double dis = 0;
	for (int i = 0; i < 6; ++i)
	{
		dis += pow(realPos[i] - pos[i], 2);
	}
	if (dis < 0.001) return true;
	else return false;
}

Vector3d Calibration::getKVector(Matrix4d matrix)
{
	double theta = acos((matrix(0, 0) + matrix(1, 1) + matrix(2, 2) - 1) / 2);

	Vector3d k;
	k(0) = (matrix(2, 1) - matrix(1, 2)) / 2 / sin(theta) * theta;
	k(1) = (matrix(0, 2) - matrix(2, 0)) / 2 / sin(theta) * theta;
	k(2) = (matrix(1, 0) - matrix(0, 1)) / 2 / sin(theta) * theta;
	return k;
}

void Calibration::getSeriesKVector(int num, Vector3d & KA1, Vector3d & KA2, Vector3d & KB1,
	Vector3d & KB2, Matrix4d& MA1, Matrix4d& MA2, Matrix4d& MB1, Matrix4d& MB2)
{
	MA1 = matrixRobotCali[0 + 3 * num].inverse()*matrixRobotCali[1 + 3 * num];
	MA2 = matrixRobotCali[1 + 3 * num].inverse()*matrixRobotCali[2 + 3 * num];
	MB1 = matrixEndBase[0 + 3 * num] * matrixEndBase[1 + 3 * num].inverse();
	MB2 = matrixEndBase[1 + 3 * num] * matrixEndBase[2 + 3 * num].inverse();

	KA1 = getKVector(MA1);
	KA2 = getKVector(MA2);
	KB1 = getKVector(MB1);
	KB2 = getKVector(MB2);
}

Vector3d Calibration::getT(Matrix4d A, Matrix4d B, Matrix3d R)
{
	Matrix3d E;
	Matrix3d RA = A.block(0, 0, 3, 3);
	Vector3d TA = A.block(0, 3, 3, 1);
	Vector3d TB = B.block(0, 3, 3, 1);

	E << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	return (RA - E).inverse()*(R*TB - TA);
}

Matrix4d Calibration::calibrationMatrix()
{
	Vector3d KA[6], KB[6];
	Matrix4d MA[6], MB[6];
	for (int i = 0; i < 3; i++)
	{
		getSeriesKVector(i, KA[2 * i + 0], KA[2 * i + 1], KB[2 * i + 0], KB[2 * i + 1],
			MA[2 * i + 0], MA[2 * i + 1], MB[2 * i + 0], MB[2 * i + 1]);
	}

	Matrix39d A, B;
	A.block(0, 0, 3, 1) = KA[0];
	A.block(0, 1, 3, 1) = KA[1];
	A.block(0, 2, 3, 1) = KA[0].cross(KA[1]);
	A.block(0, 3, 3, 1) = KA[2];
	A.block(0, 4, 3, 1) = KA[3];
	A.block(0, 5, 3, 1) = KA[2].cross(KA[3]);
	A.block(0, 6, 3, 1) = KA[4];
	A.block(0, 7, 3, 1) = KA[5];
	A.block(0, 8, 3, 1) = KA[4].cross(KA[5]);

	B.block(0, 0, 3, 1) = KB[0];
	B.block(0, 1, 3, 1) = KB[1];
	B.block(0, 2, 3, 1) = KB[0].cross(KB[1]);
	B.block(0, 3, 3, 1) = KB[2];
	B.block(0, 4, 3, 1) = KB[3];
	B.block(0, 5, 3, 1) = KB[2].cross(KB[3]);
	B.block(0, 6, 3, 1) = KB[4];
	B.block(0, 7, 3, 1) = KB[5];
	B.block(0, 8, 3, 1) = KB[4].cross(KB[5]);

	Matrix3d R = A * B.transpose()*(B*B.transpose()).inverse();

	Matrix183d A1;
	Vector18d B1;
	Matrix3d E;
	E << 1, 0, 0, 0, 1, 0, 0, 0, 1;

	for (int i = 0; i < 6; i++)
	{
		A1.block(3 * i, 0, 3, 3) = MA[i].block(0, 0, 3, 3) - E;
		B1.block(3 * i, 0, 3, 1) = R * MB[i].block(0, 3, 3, 1) - MA[i].block(0, 3, 3, 1);
	}
	Vector3d T = (A1.transpose()*A1).inverse()*(A1.transpose()*B1);

	Matrix4d result;
	result << R, T, 0, 0, 0, 1;
	return result;
}

void Calibration::OnCalibration()
{
	ofstream m_posFile("..\\src\\data\\posData.txt");
	ofstream m_refFile("..\\src\\data\\refData.txt");
	ofstream m_caliFile("..\\src\\data\\robotCaliData.txt");
	if (!m_posFile.is_open())
	{
		cout << "can not open pos file" << endl;
		return;
	}
	if (!m_refFile.is_open())
	{
		cout << "can not open ref file" << endl;
		return;
	}
	if (!m_caliFile.is_open())
	{
		cout << "can not open cali file" << endl;
		return;
	}

	for (int i = 0; i < matrixEndBase.size(); i++)
	{
		m_posFile << matrixEndBase[i] << endl << endl;
		m_refFile << matrixRobotCali[i] << endl << endl;
	}
	m_posFile.close();
	m_refFile.close();

	caliMatrix = calibrationMatrix();
	m_caliFile << caliMatrix << endl;
	m_caliFile.close();
	cout << "robot cali matrix" << endl;
	cout << caliMatrix << endl;

	isCalibrated = true;
}

void Calibration::OnCollection()
{
	static int pointNum = 0;

	Matrix4d refMatrix;
	if (!m_device->getToolTransformationMatrix(robotRef, caliRef, refMatrix))
	{
		ui.textBrowser->append("collect failed!");
		return;
	}
	refMatrix(0, 3) /= 1000;
	refMatrix(1, 3) /= 1000;
	refMatrix(2, 3) /= 1000;
	matrixRobotCali.push_back(refMatrix);
	cout << "ref matrix" << endl;
	cout << refMatrix << endl;

	double pos[6], posMat[4][4];
	m_robot->GetTCPPos(pos);
	for (int i = 0; i < 6; i++)
	{
		cout << pos[i] << "  ";
	}
	cout << endl;

	m_robot->UR6params_2_matrix(pos, posMat);
	Matrix4d robotMatrix;
	robotMatrix = mat2Matrix4d(posMat);
	matrixEndBase.push_back(robotMatrix);
	cout << "robot matrix" << endl;
	cout << robotMatrix << endl;

	ui.textBrowser->append("collect " + QString::number(pointNum + 1) + " point!");
	pointNum++;
}

void Calibration::OnAuto()
{
	matrixEndBase.clear();
	matrixRobotCali.clear();
	ifstream m_posFile("..\\src\\data\\posData.txt");
	if (!m_posFile.is_open())
	{
		cout << "can not open pos file" << endl;
		return;
	}

	for (int i = 0; i < 9; ++i)
	{
		Matrix4d refMatrix, posMatrix;
		for (int j = 0; j < 4; ++j)
		{
			for (int k = 0; k < 4; ++k)
			{
				m_posFile >> posMatrix(j, k);
			}
		}
		matrixEndBase.push_back(posMatrix);
	}
	m_posFile.close();

	for (int i = 0; i < 9; ++i)
	{
		double pos[6], mat[4][4];
		Matrix4d2mat(matrixEndBase[i], mat);
		m_robot->matrix_2_UR6params(mat, pos);
		m_robot->Movel_pose(pos, 3, 0.5);
		while (!isReach(pos)) m_robot->Movel_pose(pos, 3, 0.5);
		Matrix4d refMatrix;
		m_device->getToolTransformationMatrix(robotRef, caliRef, refMatrix);
		matrixRobotCali.push_back(refMatrix);
	}
}

void Calibration::OnLoadData()
{
	ifstream m_caliFile("..\\src\\data\\robotCaliData.txt");
	if (!m_caliFile.is_open())
	{
		cout << "can not open cali file" << endl;
		return;
	}

	for (int j = 0; j < 4; ++j)
	{
		for (int k = 0; k < 4; ++k)
		{
			m_caliFile >> caliMatrix(j, k);
		}
	}

	cout << "robot cali matrix" << endl;
	cout << caliMatrix << endl;

	isCalibrated = true;
}

bool Calibration::isCalibrationFinished()
{
	return isCalibrated;
}