// reading_in.cpp : Defines the entry point for the console application.
//



#include <iostream>
#include <fstream>
#include <sstream>
#include "DepthProcesser.h"
#include "ICP.h"
#include "icpPointToPlane.h"
#include "icpPointToPoint.h"
using namespace std;
size_t width = 1, height = 1;

unsigned short *tomb;
void readOneExample(int framenumber) {
	int a;
	
	std::ostringstream oss;
	oss << framenumber;
	std::ifstream infile("frames/example" + oss.str() + ".txt");

	if (infile) {

		int counter = 0;
		while (infile >> a)
		{
			if (counter == 0) {
				width = a;

			}
			else if (counter == 1) {
				height = a;
				tomb = new unsigned short[width*height];
			}
			else {
				tomb[counter - 2] = a;
			}
			counter++;
		}
		infile.close();
		std::cout << "sikerult a "<<framenumber<<". fajlt beolvasni"<<std::endl;
	}
	else {
		std::cout << "nem iskerult a fajlt beolvasni" << std::endl;
		return;
	}
}



int main() {
	int indexek[] = {
		0,2,4,7,8,9,10,11,12,13
	};
	

	std::cout << "Program elindult" << endl;
	DepthProcesser depthproc;
	//depthproc.InitialFromFile(10);
	//depthproc.ClearKimenet();
	depthproc.WriteOutPontok(100);
	for (int i = 23; i < 37; i++) {
		std::cout << i << ". fajl:" << std::endl;
		readOneExample(i);
		std::cout << "	process begin" << std::endl;
		depthproc.Process(tomb, width, height, i-23);
		std::cout << "	process end" << std::endl;
		std::cout << "		pontok kiras kezdodik" << std::endl;
		depthproc.WriteOutPontok(i);
		delete[] tomb;
	}
	depthproc.VizsszintbeHozas();
	
	depthproc.WriteOutPontok(101);
	/* Marching Cubes */
	std::cout << "The Math has begin" << std::endl;
	depthproc.MarchingCubes();
	std::cout << "The Math has benn done" << std::endl;
	/* kiírás obj-be */
	depthproc.WriteOutHaromszogek();
	std::cout << "Written Out" << std::endl;
	
	std::cout << "Vegzett" << std::endl;
	int a;
	std::cin >> a;
	return 0;
}


