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

	void setHostname(string hostname);	//设置设备的名称，默认为"P9-02926"
	bool initDevice();	//初始化导航头
	bool loadTool(const char* toolFilePath, int& portHandle);		//加载参考架，返回值：参考架的标号
	bool initTools();	//初始化所有参考架
	bool startTracking();	//开始跟踪
	bool stopTracking();	//停止跟踪
	bool isToolMissing(int portHandle);	//返回值：	1：看不到参考架，0：看到参考架

	//每载入一个rom文件，就会产生一个与portHandle对应的偏离矩阵，初始为单位阵
	//可通过函数void setDeviateMatrix(int, vtkMatrix4x4* )设置
	map<int, Matrix4d> m_deviateMatrix;
	//设置工具（portHandle）的偏离矩阵,从虚拟工具坐标系到实际工具坐标系
	void setDeviateMatrix(int portHandle, Matrix4d matrix);
	// matrix from tool to NDI world
	bool getToolMatrix(int portHandle, Matrix4d& matrix);
	//tool原点在NDI世界坐标系下的坐标
	bool getToolOrigin(int portHandle, Vector3d& point);

	// matrix from tool1(portHandl1) to tool2(portHandle2)
	bool getToolTransformationMatrix(int portHandle1, int portHandle2, Matrix4d& matrix);
	// 工具1的原点在工具2坐标系下的坐标
	bool getToolTransformationOrigin(int portHandle1, int portHandle2, Vector3d& point);

private:
	CombinedApi m_capi;
	bool apiSupportsBX2;

	void determineApiSupportForBX2();
	Matrix4d TranformtoMatrix(Transform transform);
};
