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

void Calibration::getSeriesKVector(int num, int size, Vector3d& KA, Vector3d& KB, Matrix4d& MA, Matrix4d& MB)
{
	MA = matrixRobotCali[size + num].inverse()*matrixRobotCali[num];
	MB = matrixEndBase[size + num] * matrixEndBase[num].inverse();
	KA = getKVector(MA);
	KB = getKVector(MB);
}

Matrix4d Calibration::calibrationMatrix()
{
	int num = matrixEndBase.size() / 2;
	vector<Matrix4d> MA(num), MB(num);//矩阵MA和MB
	vector<Vector3d> KA(num), KB(num);//对应的对数映射KA，KB

	for (int i = 0; i < num; i++) {
		getSeriesKVector(i, num, KA[i], KB[i], MA[i], MB[i]);
	}

	//求旋转矩阵
	Matrix3d M;
	M.Zero();
	for (int i = 0; i < num; i++) {
		M += KB[i] * KA[i].transpose();
	}

	Eigen::EigenSolver<Matrix3d> ES(M.transpose()*M);
	Matrix3d D = ES.pseudoEigenvalueMatrix();
	Matrix3d V = ES.pseudoEigenvectors();
	for (int i = 0; i < 3; i++) {
		D(i, i) = sqrt(D(i, i));
	}

	Matrix3d R = (V * D * V.inverse()).inverse() * M.transpose();

	//求平移向量
	Matrix3d E;
	E << 1, 0, 0, 0, 1, 0, 0, 0, 1;

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;
	A.resize(3 * num, 3);
	for (int i = 0; i < num; i++) {
		A.block(3 * i, 0, 3, 3) = MA[i].block(0, 0, 3, 3) - E;
	}

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B;
	B.resize(3 * num, 1);
	for (int i = 0; i < num; i++) {
		B.block(3 * i, 0, 3, 1) = R * MB[i].block(0, 3, 3, 1) - MA[i].block(0, 3, 3, 1);
	}

	Vector3d T = (A.transpose()*A).inverse()*A.transpose()*B;

	Matrix4d caliMatrix;
	caliMatrix << R, T, 0, 0, 0, 1;
	caliMatrix(0, 3) /= 1000;
	caliMatrix(1, 3) /= 1000;
	caliMatrix(2, 3) /= 1000;

	return caliMatrix;
}

void Calibration::OnCalibration()
{
	ofstream m_posFile("..\\data\\posData.txt");
	ofstream m_refFile("..\\data\\refData.txt");
	ofstream m_caliFile("..\\data\\robotCaliData.txt");
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
	//refMatrix(0, 3) /= 1000;
	//refMatrix(1, 3) /= 1000;
	//refMatrix(2, 3) /= 1000;
	matrixRobotCali.push_back(refMatrix);
	cout << "ref matrix" << endl;
	cout << refMatrix << endl;

	double pos[6] = { 0 };
	double posMat[4][4];
	m_robot->GetTCPPos(pos);
	for (int i = 0; i < 6; i++)
	{
		cout << pos[i] << "  ";
	}
	cout << endl;

	m_robot->UR6params_2_matrix(pos, posMat);
	Matrix4d robotMatrix;
	robotMatrix = mat2Matrix4d(posMat);
	robotMatrix(0, 3) *= 1000;
	robotMatrix(1, 3) *= 1000;
	robotMatrix(2, 3) *= 1000;
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
	ifstream m_posFile("..\\data\\posData.txt");
	if (!m_posFile.is_open())
	{
		cout << "can not open pos file" << endl;
		return;
	}

	while(1)
	{
		if (m_posFile.eof()) {
			break;
		}

		Matrix4d posMatrix;
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

	for (int i = 0; i < matrixEndBase.size(); ++i)
	{
		double pos[6], mat[4][4];
		Matrix4d2mat(matrixEndBase[i], mat);
		m_robot->matrix_2_UR6params(mat, pos);
		m_robot->Movel_pose(pos, 3, 0.5);
		while (!isReach(pos)); //m_robot->Movel_pose(pos, 3, 0.5);
		Matrix4d refMatrix;
		m_device->getToolTransformationMatrix(robotRef, caliRef, refMatrix);
		matrixRobotCali.push_back(refMatrix);
	}
}

void Calibration::OnLoadData()
{
	ifstream m_caliFile("..\\data\\robotCaliData.txt");
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