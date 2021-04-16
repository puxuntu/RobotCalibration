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

void Calibration::getSeriesKVector(int num, Vector3d& ka, Vector3d& kb, Matrix4d& ma, Matrix4d& mb)
{
	ma = matrixRobotCali[1 + 2 * num].inverse()*matrixRobotCali[0 + 2 * num];
	mb = matrixEndBase[1 + 2 * num] * matrixEndBase[0 + 2 * num].inverse();
	ka = getKVector(ma);
	kb = getKVector(mb);
}

Matrix4d Calibration::calibrationMatrix()
{
	int num = matrixEndBase.size() / 2;
	vector<Matrix4d> ma(num), mb(num);//矩阵MA和MB
	vector<Vector3d> ka(num), kb(num);//对应的对数映射KA，KB

	for (int i = 0; i < num; i++) {
		getSeriesKVector(i, ka[i], kb[i], ma[i], mb[i]);
	}

	Matrix3d m;
	m.Zero();
	for (int i = 0; i < num; i++) {
		m += kb[i] * ka[i].transpose();
	}

	Eigen::EigenSolver<Matrix3d> es(m.transpose()*m);
	Matrix3d d = es.pseudoEigenvalueMatrix();
	Matrix3d v = es.pseudoEigenvectors();
	for (int i = 0; i < 3; i++) {
		d(i, i) = sqrt(d(i, i));
	}

	//旋转矩阵
	Matrix3d r = (v * d * v.inverse()).inverse() * m.transpose();

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a;
	a.resize(3 * num, 3);
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> b;
	b.resize(3 * num, 1);

	Vector3d t = (a.transpose()*a).inverse()*a.transpose()*b;

	Matrix4d ans;
	ans << r, t, 0, 0, 0, 1;
	return ans;
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
		while (!isReach(pos)) m_robot->Movel_pose(pos, 3, 0.5);
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