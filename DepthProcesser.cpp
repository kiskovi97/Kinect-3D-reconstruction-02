#include "DepthProcesser.h"
#include <vector>
#include "icpPointToPoint.h"
#include <time.h>
#include <string>
#include "icpPointToPlane.h"


typedef unsigned short uint16;
typedef unsigned int uint32;

DepthProcesser::DepthProcesser()
{
	cam.position = vec3(0, 0, 0);
	nezo.position = vec3(0, 0, 0);

	Forgatas = Matrix::eye(3);
	Eltolas = Matrix(3, 1);
	
	array3D.resize(WIDTH);
	for (int i = 0; i < WIDTH; ++i) {
		array3D[i].resize(HEIGHT);
		for (int j = 0; j < HEIGHT; ++j) {
			array3D[i][j].resize(DEPTH);
			for (int k = 0; k < DEPTH; ++k)
				array3D[i][j][k] = 0;
		}
			
	}
	kimenet.resize(WIDTH);
	for (int i = 0; i < WIDTH; ++i) {
		kimenet[i].resize(HEIGHT);
		for (int j = 0; j < HEIGHT; ++j) {
			kimenet[i][j].resize(DEPTH);
			for (int k = 0; k < DEPTH; ++k)
				kimenet[i][j][k] = 0;
		}

	}
}

void DepthProcesser::InitialFromFile(int frameNumber) {
	eddigipontok.clear();
	elozopontok.clear();
	pontok.clear();
	double a;
	double b;
	double c;
	std::ostringstream oss;
	oss << frameNumber;
	std::ifstream infile(INITIALLOG + oss.str() + ".obj");
	if (infile) {
		std::string s;
		infile >> s;
		infile >> s;
		infile >> s;
		infile >> s;
		infile >> s;
		infile >> s;
		infile >> s;
		while (infile >> a)
		{
			infile >> b;
			infile >> c;
			vec3 pont = vec3(a, b, c);
			int XFaktor = (WIDTH_MAX / WIDTH) * 2;
			int YFaktor = (HEIGHT_MAX / HEIGHT) * 2;
			int ZFaktor = (DEPTH_MAX / DEPTH);
			if (pont.z > 0 && pont.z < DEPTH_MAX && pont.x<WIDTH_MAX && pont.y<HEIGHT_MAX && pont.x>WIDTH_MIN && pont.y>HEIGHT_MIN) {
				int x = pont.x / XFaktor + WIDTH / 2;
				int y = pont.y / YFaktor + HEIGHT / 2;
				int z = pont.z / ZFaktor;
				eddigipontok.push_back(pont);
				kimenet[x][y][z] ++;
			}
			if (!(infile >> s)) break;
		}
		infile.close();
		std::cout << "sikerult a " << frameNumber << ". fajlt beolvasni" << std::endl;
	}
	else {
		std::cout << "ERROR: nem iskerult a fajlt beolvasni" << std::endl;
		return;
	}
}

DepthProcesser::~DepthProcesser()
{
	delete[] Elozoframe;
	delete[] Mostaniframe;
}

void DepthProcesser::ClearKimenet()
{
	std::cout << "		pont haloba torlese" << std::endl;
	for (int i = 0; i < WIDTH; ++i) {
		for (int j = 0; j < HEIGHT; ++j) {
			for (int k = 0; k < DEPTH; ++k)
				kimenet[i][j][k] = 0;
		}
	}
	std::cout << "		kimenet smootholasa" << std::endl;
	for (int i = 0; i < eddigipontok.size(); i++)
	{
		vec3 pont1 = eddigipontok.at(i);
		for (int j = i+1; j < eddigipontok.size(); j++)
		{
			vec3 pont2 = eddigipontok.at(j);
			if ((pont2 - pont1).Length() < SMOOTH) {
				pont1 = pont1 + (pont2 - pont1)*SMOOTHALPHA;
			}
		}
		eddigipontok.at(i) = pont1;
		int XFaktor = (WIDTH_MAX / WIDTH) * 2;
		int YFaktor = (HEIGHT_MAX / HEIGHT) * 2;
		int ZFaktor = (DEPTH_MAX / DEPTH);
		if (pont1.z > 0 && pont1.z < DEPTH_MAX && pont1.x<WIDTH_MAX && pont1.y<HEIGHT_MAX && pont1.x>WIDTH_MIN && pont1.y>HEIGHT_MIN) {
			int x = pont1.x / XFaktor + WIDTH / 2;
			int y = pont1.y / YFaktor + HEIGHT / 2;
			int z = pont1.z / ZFaktor;
			kimenet[x][y][z] ++;
		}
	}
}

// ez megy vegig minden kockan
void DepthProcesser::MarchingCubes() {

	for (int i = 0; i < (WIDTH-1); i++)
		for (int j = 0; j < (HEIGHT-1); j++)
			for (int k = 0; k < (DEPTH-1); k++)
			{
				MCube(i,j,k);
			}

}

//Ez a fuggveny generalja le egy darab kockara a haromszogeket
void DepthProcesser::MCube(int x, int y, int z) {
	bool A = kimenet[x][y][z] > SURUSEG_MARCHINGCUBES;
	bool B = kimenet[x+1][y][z] > SURUSEG_MARCHINGCUBES;
	bool C = kimenet[x+1][y][z+1] > SURUSEG_MARCHINGCUBES;
	bool D = kimenet[x][y][z+1] > SURUSEG_MARCHINGCUBES;
	bool E = kimenet[x][y+1][z] > SURUSEG_MARCHINGCUBES;
	bool F = kimenet[x+1][y+1][z] > SURUSEG_MARCHINGCUBES;
	bool G = kimenet[x+1][y+1][z+1] > SURUSEG_MARCHINGCUBES;
	bool H = kimenet[x][y+1][z+1] > SURUSEG_MARCHINGCUBES;

	std::vector<int> triangles = mcb.Haromszogek(A + B * 2 + C * 4 + D * 8 + E * 16 + F * 32 + G * 64 + H * 128);
	if (triangles.size()>0)
	for (int i = 0; i < (triangles.size()); i+=3)
	{
		std::vector<vec3> har;
		har.push_back(vec3(x, y, z) + eltolas[triangles[i]]);
		har.push_back(vec3(x, y, z) + eltolas[triangles[i+1]]);
		har.push_back(vec3(x, y, z) + eltolas[triangles[i+2]]);
		vec3 a = triangles[i] - triangles[i + 1];
		vec3 b = triangles[i+1] - triangles[i + 2];
		vec3 norm;
		norm.x = a.y*b.z - a.z*b.y;
		norm.y = a.z * b.x - a.x*b.z;
		norm.z = a.x*b.y - a.y*b.x;
		har.push_back(norm.normalize());
		haromszogek.push_back(har);
	}
	
}

// fileba kiirasa a meglevo haromszogeknek
void DepthProcesser::WriteOutHaromszogek() {
	std::ofstream myfile;
	myfile.open("Haromszogek.obj");
	myfile << "# Scanner_file" << std::endl;
	myfile << "mtllib untitled.mtl" << std::endl;
	myfile << "o ScanObject" << std::endl;

	for (std::vector<vec3> har : haromszogek)
	{
		myfile << "v " << har[0].x << " " << har[0].y << " " << har[0].z << std::endl;
		myfile << "v " << har[1].x << " " << har[1].y << " " << har[1].z << std::endl;
		myfile << "v " << har[2].x << " " << har[2].y << " " << har[2].z << std::endl;
	}
	myfile << "usemtl Material01" << std::endl;
	myfile << "s 1" << std::endl;
	int i = 0;
	for (std::vector<vec3> har : haromszogek)
	{
		myfile << "f " << i*3 +1 << "//" << i+1 << " "<< i * 3 +2<< "//" << i + 1 <<" "<< i * 3 +3 << "//" << i + 1 << std::endl;
		i++;
	}

	myfile.close();
}
void DepthProcesser::AddNewPoint() {
	std::cout << "		pont haloba torlese" << std::endl;
	for (int i = 0; i < WIDTH; ++i) {
		for (int j = 0; j < HEIGHT; ++j) {
			for (int k = 0; k < DEPTH; ++k)
				kimenet[i][j][k] = 0;
		}
	}
	std::cout << "		elozopontok haloba szervezese" << std::endl;
	for (int i = 0; i < eddigipontok.size(); i++) {
		vec3 point = eddigipontok.at(i);
		int XFaktor = (WIDTH_MAX / WIDTH) * 2;
		int YFaktor = (HEIGHT_MAX / HEIGHT) * 2;
		int ZFaktor = (DEPTH_MAX / DEPTH);
		if (point.z > 0 && point.z < DEPTH_MAX && point.x<WIDTH_MAX && point.y<HEIGHT_MAX && point.x>WIDTH_MIN && point.y>HEIGHT_MIN) {
			int x = point.x / XFaktor + WIDTH / 2;
			int y = point.y / YFaktor + HEIGHT / 2;
			int z = point.z / ZFaktor;
			kimenet[x][y][z] ++;
		}
	}
	std::cout << "		mostanipontok haloba szervezese" << std::endl;
	for (int i = 0; i < pontok.size(); i++) {
		vec3 point = pontok.at(i);
		int XFaktor = (WIDTH_MAX / WIDTH) * 2;
		int YFaktor = (HEIGHT_MAX / HEIGHT) * 2;
		int ZFaktor = (DEPTH_MAX / DEPTH);
		if (point.z > 0 && point.z < DEPTH_MAX && point.x<WIDTH_MAX && point.y<HEIGHT_MAX && point.x>WIDTH_MIN && point.y>HEIGHT_MIN) {
			int x = point.x / XFaktor + WIDTH / 2;
			int y = point.y / YFaktor + HEIGHT / 2;
			int z = point.z / ZFaktor;
			if (kimenet[x][y][z] < SURUSEG_NEWPONTOK)
			{
				eddigipontok.push_back(point);
			}
			kimenet[x][y][z] ++;
			
		}
	}
	
	
}
void DepthProcesser::VizsszintbeHozas() {
	int maxDepthI = 0;
	int minDepthI = 0;
	if (eddigipontok.empty()) {
		std::cout << "Nem sikerult a Vizszintesbe hozas, mert nincsenek pontok" << std::endl;
		return;
	}
	for (int i = 0; i < eddigipontok.size(); i++)
	{
		if (eddigipontok.at(i).z>eddigipontok.at(maxDepthI).z) maxDepthI = i;
		if (eddigipontok.at(i).z < eddigipontok.at(minDepthI).z) minDepthI = i;
	}
		
	vec3 legtavolabbai = eddigipontok.at(maxDepthI);
	vec3 legkozelebbi = eddigipontok.at(minDepthI);
	vec3 irany = legkozelebbi - legtavolabbai;
	float tanalpha = irany.y / irany.z;
	float alpha = atanf(tanalpha);
	
	Matrix R = Matrix::eye(3);
	
	R.val[1][1] = cosf(alpha);
	R.val[2][1] = sinf(alpha);
	R.val[2][2] = cosf(alpha);
	R.val[1][2] = -sinf(alpha);
	for (int i = 0; i < HEIGHT; ++i)
		for (int j = 0; j < WIDTH; ++j)
			for (int k = 0; k < DEPTH; ++k)
				kimenet[i][j][k] = 0;
	for (int i = 0; i < eddigipontok.size(); i++)
	{
		vec3 pont = eddigipontok.at(i);
		float x = pont.x;
		float y = pont.y;
		float z = pont.z;
		pont.x = x*R.val[0][0] + y*R.val[0][1] + z*R.val[0][2];
		pont.y = x*R.val[1][0] + y*R.val[1][1] + z*R.val[1][2] +100;
		pont.z = x*R.val[2][0] + y*R.val[2][1] + z*R.val[2][2];
		eddigipontok.at(i).x = pont.x;
		eddigipontok.at(i).y = pont.y;
		eddigipontok.at(i).z = pont.z;
		
		int XFaktor = (WIDTH_MAX / WIDTH) * 2;
		int YFaktor = (HEIGHT_MAX / HEIGHT) * 2;
		int ZFaktor = (DEPTH_MAX / DEPTH);
		if (pont.z > 0 && pont.z < DEPTH_MAX && pont.x<WIDTH_MAX && pont.y<HEIGHT_MAX && pont.x>WIDTH_MIN && pont.y>HEIGHT_MIN) {
			int x = pont.x / XFaktor + WIDTH / 2;
			int y = pont.y / YFaktor + HEIGHT / 2;
			int z = pont.z / ZFaktor;
			kimenet[x][y][z] ++;
		}
	}
	
	

}
void DepthProcesser::WriteOutPontok(int i) {
	std::ofstream myfile;
	std::string s = std::to_string(i);
	myfile.open("ElozoPontok"+s+".obj");
	myfile << "# Scanner_file" << std::endl;
	myfile << "mtllib untitled.mtl" << std::endl;
	myfile << "o ScanObject" << std::endl;

	for (vec3 pont : elozopontok)
	{
		myfile << "v " << pont.x << " " << pont.y << " " << pont.z << std::endl;
	}
	myfile.close();

	std::ofstream myfile2;
	std::string s2 = std::to_string(i);
	myfile2.open("TeljesPontok" + s2 + ".obj");
	myfile2 << "# Scanner_file" << std::endl;
	myfile2 << "mtllib untitled.mtl" << std::endl;
	myfile2 << "o ScanObject" << std::endl;

	for (vec3 pont : eddigipontok)
	{
		myfile2 << "v " << pont.x << " " << pont.y << " " << pont.z << std::endl;
	}
	myfile2.close();
}

void DepthProcesser::Process(uint16* depthfield,const int m_depthWidth,const int m_depthHeight, int frame_number) {
	float m_Depth;
	// kamera kalibralasa
	{
		//2.8 fok =  0.397935 degree
		m_Depth = m_depthHeight / (2*tanf(0.397935f));
		cam.forward = vec3(0, 0, m_Depth*1.0f);
		cam.right = vec3(m_depthWidth*0.5f, 0, 0);
		cam.up = vec3(0, m_depthHeight*0.5f, 0);
	}
	// a kinect depth mapja alapjan kiszamolt pontok
	for (int i = 0; i < m_depthWidth * m_depthHeight; ++i)
	{
		// pixel helyenek, a pont iranyanak es helyenek kiszamitasa kiszamitasa
		int x = i % m_depthWidth;
		int y = (i - x) / m_depthWidth;
		vec3 temp_pont = cam.Pixel(x - (m_depthWidth / 2), y - (m_depthHeight / 2), m_depthWidth, m_depthHeight);
		vec3 irany = temp_pont - cam.position;
		float my = depthfield[i]*1.0f * ((y - (m_depthHeight / 2))*(-1.0f) / (2.0f * m_Depth));
		float mx = depthfield[i]*1.0f * ((x - (m_depthWidth / 2))*(1.0f) / (2.0f * m_Depth));
		irany=irany.normalize();
		vec3 pont = cam.forward.normalize()*(depthfield[i] / 3.0f) + cam.right.normalize()*mx + cam.up.normalize()*my + cam.position;
		// pontok helyenek kerekitese egy adott racsra
		
		int XFaktor = (WIDTH_MAX / WIDTH) * 2;
		int YFaktor = (HEIGHT_MAX / HEIGHT) * 2;
		int ZFaktor = (DEPTH_MAX / DEPTH) ;
		if (pont.z >0 && pont.z < DEPTH_MAX && pont.x<WIDTH_MAX && pont.y<HEIGHT_MAX && pont.x>WIDTH_MIN && pont.y>HEIGHT_MIN){
			int x = pont.x / XFaktor + WIDTH/2;
			int y = pont.y / YFaktor + HEIGHT/2;
			int z = pont.z / ZFaktor;
			if (array3D[x][y][z]  >= SURUSEG_SZURESELEJEN ) pontok.push_back(pont);
			array3D[x][y][z] ++;
		}
	}
	
	time_t rawtime;
	struct tm * timeinfo;

	// ICP kiszamolasa
	
	if (frame_number > 0)
	{
		std::cout << "		Pontok athelyezse" << std::endl;
		delete[] Elozoframe;
		delete[] Mostaniframe;
		
		Elozoframe = new double[elozopontok.size()*3];
		Mostaniframe = new double[3 * pontok.size()];
		for (int i = 0; i < pontok.size(); i++)
		{
			Mostaniframe[i * 3] = pontok[i].x;
			Mostaniframe[i * 3 + 1] = pontok[i].y;
			Mostaniframe[i * 3 + 2] = pontok[i].z;
		}
		std::cout << "		ICP Inicializalasa" << std::endl;
		int iteration = 0;;
		double hiba = 1.0;
		while ((iteration < ICPITERATION) && (abs(hiba) > HIBATURES)) {
			
			for (int i = 0; i < elozopontok.size(); i++)
			{
				Elozoframe[i * 3] = elozopontok.at(i).x;
				Elozoframe[i * 3 + 1] = elozopontok.at(i).y;
				Elozoframe[i * 3 + 2] = elozopontok.at(i).z;
			}
			Matrix R = Matrix::eye(3);
			Matrix t(3, 1);
			// ICP kiszamitasa
			time(&rawtime);
			timeinfo = localtime(&rawtime);
			printf("Current local time and date: %s", asctime(timeinfo));
			std::cout <<"---- "<< iteration << ". iteration kovetkezik" << std::endl;
			IcpPointToPoint icp(Mostaniframe, pontok.size(), 3);
			double residual = icp.fit(Elozoframe, elozopontok.size(), R, t, ICP_PONTOK);
			std::cout <<"---- "<< iteration << ". iteration vegzett" << std::endl;

			std::cout << std::endl << t << std::endl;
			std::cout << std::endl << R << std::endl;
			
			hiba = abs(t.val[0][0]) + abs(t.val[1][0]) + abs(t.val[2][0]);
			hiba += (abs(R.val[0][1]) + abs(R.val[0][2]) + abs(R.val[1][2]))*10;
			
			std::cout << "hiba: " << hiba << std::endl;
			std::cout << "A transzformacio alkalmazasa"<< std::endl;
			for (int i = 0; i < elozopontok.size(); i++) {
				vec3 pont = elozopontok.at(i);

				float x = pont.x;
				float y = pont.y;
				float z = pont.z;
				pont.x = x*R.val[0][0] + y*R.val[0][1] + z*R.val[0][2];
				pont.y = x*R.val[1][0] + y*R.val[1][1] + z*R.val[1][2];
				pont.z = x*R.val[2][0] + y*R.val[2][1] + z*R.val[2][2];
				pont.x = pont.x + t.val[0][0];
				pont.y = pont.y + t.val[1][0];
				pont.z = pont.z + t.val[2][0];

				elozopontok.at(i).x = pont.x;
				elozopontok.at(i).y = pont.y;
				elozopontok.at(i).z = pont.z;

			}
			for (int i = 0; i < eddigipontok.size(); i++) {
				vec3 pont = eddigipontok.at(i);
				
				float x = pont.x;
				float y = pont.y;
				float z = pont.z;
				pont.x = x*R.val[0][0] + y*R.val[0][1] + z*R.val[0][2];
				pont.y = x*R.val[1][0] + y*R.val[1][1] + z*R.val[1][2];
				pont.z = x*R.val[2][0] + y*R.val[2][1] + z*R.val[2][2];
				pont.x = pont.x + t.val[0][0];
				pont.y = pont.y + t.val[1][0];
				pont.z = pont.z + t.val[2][0];

				eddigipontok.at(i).x = pont.x;
				eddigipontok.at(i).y = pont.y;
				eddigipontok.at(i).z = pont.z;

			}
			iteration++;
			
		}

		elozopontok.clear();
		for (int i = 0; i < pontok.size(); i++) {
			elozopontok.push_back(pontok.at(i));
		}

	}



	if (frame_number == 0) {

		Mostaniframe = new double[3 * pontok.size()];
		for (int i = 0; i < pontok.size(); i++)
		{
			Mostaniframe[i * 3] = pontok[i].x;
			Mostaniframe[i * 3 + 1] = pontok[i].y;
			Mostaniframe[i * 3 + 2] = pontok[i].z;
		}
		elozo_size = pontok.size();
		Elozoframe = new double[3 * pontok.size()];
		for (int i = 0; i < pontok.size(); i++)
		{
			Elozoframe[i * 3] = pontok[i].x;
			Elozoframe[i * 3 + 1] = pontok[i].y;
			Elozoframe[i * 3 + 2] = pontok[i].z;
		}
		elozopontok.clear();
		for (int i = 0; i < pontok.size(); i++) {
			elozopontok.push_back(pontok.at(i));
		}
	}
	
	AddNewPoint();

	

	// az adatok torlese egyenlore	
	pontok.clear();
	for (int i = 0; i < HEIGHT; ++i)
		for (int j = 0; j < WIDTH; ++j) 
			for (int k = 0; k < DEPTH; ++k)
				array3D[i][j][k] = 0;
	std::cout << "		torles sikerult" << std::endl;
	}



