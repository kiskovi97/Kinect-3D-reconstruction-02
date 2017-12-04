#pragma once

#include "Camera.h"
#include <iostream>
#include <fstream>
#include "ICP.h"
#include "MarchCubes.h"
#include "matrix.h"


#define HEIGHT 100
#define WIDTH 100
#define DEPTH 100
#define HEIGHT_MAX 400
#define HEIGHT_MIN -400
#define WIDTH_MAX 400
#define WIDTH_MIN -400
#define DEPTH_MAX 800
#define SMOOTH 7
#define SMOOTHALPHA 0.5
#define SURUSEG 3
#define SURUSEG2 1
#define SURUSEG3 0
#define ICP_PONTOK 10000
#define ICPITERATION 25
#define HIBATURES 0.01

typedef unsigned short uint16;
typedef unsigned int uint32;
class DepthProcesser
{
private:
	int elozo_size=0;
	std::vector<vec3> eltolas = {
		vec3(0.5,0,0),
		vec3(1,0,0.5),
		vec3(0.5,0,1),
		vec3(0,0,0.5),
		vec3(0.5,1,0),
		vec3(1,1,0.5),
		vec3(0.5,1,1),
		vec3(0,1,0.5),
		vec3(0,0.5,0),
		vec3(1,0.5,0),
		vec3(1,0.5,1),
		vec3(0,0.5,1)
	};
	

	double *Elozoframe;
	double *Mostaniframe;
	Matrix Forgatas;
	Matrix Eltolas;

	//ICP icp;
	MarchCubes mcb;
	std::vector<vec3> pontok;
	std::vector<vec3> elozopontok;
	std::vector<vec3> eddigipontok;
	std::vector<std::vector<std::vector<int> > > array3D;
	std::vector<std::vector<std::vector<int> > > kimenet;
	std::vector<std::vector<vec3> > haromszogek;
	MyCamera cam;
	MyCamera nezo;
	bool elore = true;
	bool elso_frame = true;
	void AddNewPoint();

public:
	
	DepthProcesser();
	~DepthProcesser();
	void ClearKimenet();
	void VizsszintbeHozas();
	void WriteOutHaromszogek();
	void WriteOutPontok(int i);
	void MarchingCubes();
	void MCube(int x, int y, int z);
	
	void InitialFromFile(int frameNumber);
	void Process(uint16* depthfield, const int m_depthWidth, const int m_depthHeight, int frame_number);
};

