#include "NDI.h"

NDI::NDI()
{
	//m_hostname = "P9-02926";
	m_hostname = "COM3";
	m_capi = CombinedApi();
	apiSupportsBX2 = false;
}

NDI::~NDI()
{
}

void NDI::setHostname(string hostname)
{
	m_hostname = hostname;
}

bool NDI::initDevice()
{
	if (m_capi.connect(m_hostname) != 0)
	{
		cout << "Connection Failed!" << endl;
		return false;
	}
	Sleep(1000);
	determineApiSupportForBX2();
	if (m_capi.initialize() != 0)
	{
		cout << "Initialize Failed!" << endl;
		return false;
	}
	return true;
}

void NDI::determineApiSupportForBX2()
{
	// Lookup the API revision
	std::string response = m_capi.getApiRevision();

	// Refer to the API guide for how to interpret the APIREV response
	char deviceFamily = response[0];
	int majorVersion = m_capi.stringToInt(response.substr(2, 3));

	// As of early 2017, the only NDI device supporting BX2 is the Vega
	// Vega is a Polaris device with API major version 003
	if (deviceFamily == 'G' && majorVersion >= 3)
	{
		apiSupportsBX2 = true;
	}
}

bool NDI::loadTool(const char* toolFilePath, int& portHandle)
{
	portHandle = m_capi.portHandleRequest();
	if (portHandle < 0)
	{
		cout << "PordHandle Request Failed!";
		return false;
	}
	m_capi.loadSromToPort(toolFilePath, portHandle);

	Matrix4d matrix;
	matrix << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	m_deviateMatrix.insert(map<int, Matrix4d>::value_type(portHandle, matrix));

	return true;
}

bool NDI::initTools()
{
	vector<PortHandleInfo> portHandles = m_capi.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
	for (int i = 0; i < portHandles.size(); i++)
	{
		if (m_capi.portHandleInitialize(portHandles[i].getPortHandle()) != 0)
		{
			cout << "Init Tool Failed: " << portHandles[i].getPortHandle() << endl;
			return false;
		}
		if (m_capi.portHandleEnable(portHandles[i].getPortHandle()) != 0)
		{
			cout << "Enable Tool Failed: " << portHandles[i].getPortHandle() << endl;
			return false;
		}
	}

	// Print all enabled tools
	//portHandles = m_capi.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
	//for (int i = 0; i < portHandles.size(); i++)
	//{
	//	std::cout << portHandles[i].toString() << std::endl;
	//}
	return true;
}

bool NDI::startTracking()
{
	if (m_capi.startTracking() != 0)
	{
		return false;
	}
	return true;
}

bool NDI::stopTracking()
{
	if (m_capi.stopTracking() != 0)
	{
		return false;
	}
	return true;
}

bool NDI::isToolMissing(int portHandle)
{
	std::vector<ToolData> toolData = apiSupportsBX2 ? m_capi.getTrackingDataBX2() : m_capi.getTrackingDataBX();
	if (toolData[portHandle - 1].transform.isMissing())
	{
		return true;
	}
	return false;
}

bool NDI::getToolMatrix(int portHandle, Matrix4d& matrix)
{
	std::vector<ToolData> toolData = apiSupportsBX2 ? m_capi.getTrackingDataBX2() : m_capi.getTrackingDataBX();
	if (toolData[portHandle - 1].transform.isMissing())
	{
		return false;
	}
	matrix = TranformtoMatrix(toolData[portHandle - 1].transform)*m_deviateMatrix[portHandle];
	return true;
}

bool NDI::getToolOrigin(int portHandle, Vector3d& point)
{
	Matrix4d matrix;
	if (!getToolMatrix(portHandle, matrix))
	{
		return false;
	}
	point(0) = matrix(0, 3);
	point(1) = matrix(1, 3);
	point(2) = matrix(2, 3);
	return true;
}

bool NDI::getToolTransformationMatrix(int portHandle1, int portHandle2, Matrix4d& matrix)
{
	std::vector<ToolData> toolData = apiSupportsBX2 ? m_capi.getTrackingDataBX2() : m_capi.getTrackingDataBX();
	if (toolData[portHandle1 - 1].transform.isMissing())
	{
		cout << "tool1 is missing" << endl;
		return false;
	}
	if (toolData[portHandle2 - 1].transform.isMissing())
	{
		cout << "tool2 is missing" << endl;
		return false;
	}
	Matrix4d toolMatrix1 = TranformtoMatrix(toolData[portHandle1 - 1].transform)*m_deviateMatrix[portHandle1];
	Matrix4d toolMatrix2 = TranformtoMatrix(toolData[portHandle2 - 1].transform)*m_deviateMatrix[portHandle2];

	matrix = toolMatrix2.inverse()*toolMatrix1;
	return true;
}

bool NDI::getToolTransformationOrigin(int portHandle1, int portHandle2, Vector3d& point)
{
	Matrix4d matrix;
	if (!getToolTransformationMatrix(portHandle1, portHandle2, matrix))
	{
		return false;
	}

	point(0) = matrix(0, 3);
	point(1) = matrix(1, 3);
	point(2) = matrix(2, 3);
	return true;
}

void NDI::setDeviateMatrix(int portHandle, Matrix4d matrix)
{
	m_deviateMatrix[portHandle] = matrix;
}

Matrix4d NDI::TranformtoMatrix(Transform transform)
{
	double x, y, z, w;
	double tx, ty, tz;
	w = transform.q0;
	x = transform.qx;
	y = transform.qy;
	z = transform.qz;
	tx = transform.tx;
	ty = transform.ty;
	tz = transform.tz;

	Matrix4d matrix;

	matrix(0, 0) = 1 - 2 * y*y - 2 * z*z;
	matrix(0, 1) = 2 * x*y - 2 * z*w;
	matrix(0, 2) = 2 * x*z + 2 * y*w;
	matrix(0, 3) = tx;
	matrix(1, 0) = 2 * x*y + 2 * z*w;
	matrix(1, 1) = 1 - 2 * x*x - 2 * z*z;
	matrix(1, 2) = 2 * y*z - 2 * x*w;
	matrix(1, 3) = ty;
	matrix(2, 0) = 2 * x*z - 2 * y*w;
	matrix(2, 1) = 2 * y*z + 2 * x*w;
	matrix(2, 2) = 1 - 2 * x*x - 2 * y*y;
	matrix(2, 3) = tz;
	matrix(3, 0) = 0;
	matrix(3, 1) = 0;
	matrix(3, 2) = 0;
	matrix(3, 3) = 1;

	return matrix;
}