#pragma once
#include<iostream>
#define WIN32_LEAN_AND_MEAN
#include<Windows.h>
#include <map>
#include <Eigen/Dense>
#include "CombinedApi.h"
#include "ToolData.h"
#include "Transform.h"

#pragma comment(lib, "library.lib") 
using namespace std;
//using namespace Eigen;
typedef Eigen::Matrix4d Matrix4d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix3d Matrix3d;

class NDI
{
public:
	NDI();
	~NDI();
	string m_hostname;

	void setHostname(string hostname);	//�����豸�����ƣ�Ĭ��Ϊ"P9-02926"
	bool initDevice();	//��ʼ������ͷ
	bool loadTool(const char* toolFilePath, int& portHandle);		//���زο��ܣ�����ֵ���ο��ܵı��
	bool initTools();	//��ʼ�����вο���
	bool startTracking();	//��ʼ����
	bool stopTracking();	//ֹͣ����
	bool isToolMissing(int portHandle);	//����ֵ��	1���������ο��ܣ�0�������ο���

	//ÿ����һ��rom�ļ����ͻ����һ����portHandle��Ӧ��ƫ����󣬳�ʼΪ��λ��
	//��ͨ������void setDeviateMatrix(int, vtkMatrix4x4* )����
	map<int, Matrix4d> m_deviateMatrix;
	//���ù��ߣ�portHandle����ƫ�����,�����⹤������ϵ��ʵ�ʹ�������ϵ
	void setDeviateMatrix(int portHandle, Matrix4d matrix);
	// matrix from tool to NDI world
	bool getToolMatrix(int portHandle, Matrix4d& matrix);
	//toolԭ����NDI��������ϵ�µ�����
	bool getToolOrigin(int portHandle, Vector3d& point);

	// matrix from tool1(portHandl1) to tool2(portHandle2)
	bool getToolTransformationMatrix(int portHandle1, int portHandle2, Matrix4d& matrix);
	// ����1��ԭ���ڹ���2����ϵ�µ�����
	bool getToolTransformationOrigin(int portHandle1, int portHandle2, Vector3d& point);

private:
	CombinedApi m_capi;
	bool apiSupportsBX2;

	void determineApiSupportForBX2();
	Matrix4d TranformtoMatrix(Transform transform);
};
