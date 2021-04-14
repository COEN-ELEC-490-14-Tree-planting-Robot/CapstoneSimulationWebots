// All the webots classes are defined in the "webots" namespace


#include <iostream>
#include "httplib.h"
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Lidar.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <windows.h>
#include <map>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "Serial.h"
#include<thread>
#include<future>
#include<vector>
#include <fstream>

#define TIME_STEP 64
#define VELOCITY 8.33

using namespace webots;
using namespace std;
using namespace octomap;
using namespace httplib;
//-----------------------------
//controls
//-----------------------------
void move_4_wheels(map<string, Motor*> wheels, double v) {
	wheels["FrontLeftWheel"]->setVelocity(v * VELOCITY);
	wheels["MiddleLeftWheel"]->setVelocity(v * VELOCITY);
	wheels["BackLeftWheel"]->setVelocity(v * VELOCITY);
	wheels["FrontRightWheel"]->setVelocity(v * VELOCITY);
	wheels["MiddleRightWheel"]->setVelocity(v * VELOCITY);
	wheels["BackRightWheel"]->setVelocity(v * VELOCITY);

	wheels["MiddleLeftWheel"]->setAvailableTorque(0.0);
	wheels["MiddleRightWheel"]->setAvailableTorque(0.0);
}

void move_6_wheels(std::map<std::string, Motor*> wheels, double v) {
	wheels["MiddleRightWheel"]->setAvailableTorque(2.0);
	wheels["MiddleLeftWheel"]->setAvailableTorque(2.0);

	wheels["MiddleRightWheel"]->setVelocity(v * VELOCITY);
	wheels["MiddleLeftWheel"]->setVelocity(v * VELOCITY);
	wheels["BackRightWheel"]->setVelocity(v * VELOCITY);
	wheels["BackLeftWheel"]->setVelocity(v * VELOCITY);
	wheels["FrontRightWheel"]->setVelocity(v * VELOCITY);
	wheels["FrontLeftWheel"]->setVelocity(v * VELOCITY);
}

void turn_wheels_right(map<string, Motor*> arms) {
	arms["FrontLeftArm"]->setPosition(0.4);
	arms["FrontRightArm"]->setPosition(0.227);
	arms["BackLeftArm"]->setPosition(-0.4);
	arms["BackRightArm"]->setPosition(-0.227);
}

void turn_wheels_left(map<string, Motor*> arms) {
	arms["FrontLeftArm"]->setPosition(-0.227);
	arms["FrontRightArm"]->setPosition(-0.4);
	arms["BackLeftArm"]->setPosition(0.227);
	arms["BackRightArm"]->setPosition(0.4);
}

void wheels_straight(map<string, Motor*> arms) {
	arms["FrontLeftArm"]->setPosition(0);
	arms["FrontRightArm"]->setPosition(0);
	arms["BackLeftArm"]->setPosition(0);
	arms["BackRightArm"]->setPosition(0);
}

void turn_around(map<string, Motor*> arms, map<string, Motor*> wheels, double v) {
	arms["FrontLeftArm"]->setPosition(-0.87);
	arms["FrontRightArm"]->setPosition(0.87);
	arms["BackRightArm"]->setPosition(-0.87);
	arms["BackLeftArm"]->setPosition(0.87);

	wheels["FrontLeftWheel"]->setVelocity(v * VELOCITY);
	wheels["MiddleLeftWheel"]->setVelocity(v * VELOCITY);
	wheels["BackLeftWheel"]->setVelocity(v * VELOCITY);
	wheels["FrontRightWheel"]->setVelocity(-v * VELOCITY);
	wheels["MiddleRightWheel"]->setVelocity(-v * VELOCITY);
	wheels["BackRightWheel"]->setVelocity(-v * VELOCITY);

	wheels["MiddleLeftWheel"]->setAvailableTorque(0.0);
	wheels["MiddleRightWheel"]->setAvailableTorque(0.0);
}


//--------------Map Building---------------------------------------------------------------------
static OcTree tree(0.5);
void buildMap(Lidar* myLidar, GPS* myGps, InertialUnit* myImu, Gyro* myGyro)
{


	Pointcloud pCloud;
	const double* gps = myGps->getValues();
	point3d origin(0, 0, 0);
	const double* rpy = myImu->getRollPitchYaw();
	pose6d frameOrigin(*(gps + 1), *(gps + 2), *gps, *(rpy + 2), *rpy, *(rpy + 1));
	ofstream logfile;
	logfile.open("../../../website/Flask/static/files/logs/logs.txt");
	logfile << "Lidar scanned. Current location: X:" << *gps << ",Y:" << *(gps + 2) << ",Z:" << *(gps + 1) << ",Roll" << *(rpy) << ",Pitch:" << *(rpy + 1) << ",aw:" << *(rpy + 2) << endl;
	logfile.close();
	for (int i = 0; i < 10; i++) {
		const LidarPoint* points = myLidar->getLayerPointCloud(i);
		for (int j = 0; j < myLidar->getHorizontalResolution(); j += 2)
		{
			LidarPoint p = *(points + j);
			float d = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
			if (d > 98)
				continue;
			//cout << "x:" << p.x << ",y:" << p.y << ",z:" << p.z << endl;
			pCloud.push_back(p.y, p.z, p.x);
		}
	}

	tree.insertPointCloud(pCloud, origin, frameOrigin, -1, true, true);
	tree.updateInnerOccupancy();

	tree.writeBinary("webots.bt");
	cout << "wrote to webots.bt" << endl << endl;
}

void buildPath(Lidar* myLidar, GPS* myGps, InertialUnit* myImu, Gyro* myGyro)
{
	const double* gps = myGps->getValues();
	const double* rpy = myImu->getRollPitchYaw();
	cout << "x:" << *gps << ",y:" << *(gps + 2) << ",z:" << *(gps + 1) << ",roll" << *(rpy) << ",pitch:" << *(rpy + 1) << ",yaw:" << *(rpy + 2) << endl;
}


void print_query_info(point3d query, OcTreeNode* node) {
	if (node != NULL) {
		cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
	}
	else
		cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}

// example code
void getMap()
{
	cout << "generating example map" << endl;

	OcTree tree(0.1);  // create empty tree with resolution 0.1


	// insert some measurements of occupied cells

	for (int x = -20; x < 20; x++) {
		for (int y = -20; y < 20; y++) {
			for (int z = -20; z < 20; z++) {
				point3d endpoint((float)x * 0.05f, (float)y * 0.05f, (float)z * 0.05f);
				tree.updateNode(endpoint, true); // integrate 'occupied' measurement
			}
		}
	}

	// insert some measurements of free cells

	for (int x = -30; x < 30; x++) {
		for (int y = -30; y < 30; y++) {
			for (int z = -30; z < 30; z++) {
				point3d endpoint((float)x * 0.02f - 1.0f, (float)y * 0.02f - 1.0f, (float)z * 0.02f - 1.0f);
				tree.updateNode(endpoint, false);  // integrate 'free' measurement
			}
		}
	}

	cout << endl;
	cout << "performing some queries:" << endl;

	point3d query(0., 0., 0.);
	OcTreeNode* result = tree.search(query);
	print_query_info(query, result);

	query = point3d(-1., -1., -1.);
	result = tree.search(query);
	print_query_info(query, result);

	query = point3d(1., 1., 1.);
	result = tree.search(query);
	print_query_info(query, result);


	cout << endl;
	tree.writeBinary("simple_tree.bt");
	cout << "wrote example file simple_tree.bt" << endl << endl;
	cout << "now you can use octovis to visualize: octovis simple_tree.bt" << endl;
	cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl << endl;
}

int main(int argc, char** argv) {
	Client cli("localhost", 5000);


	// create the Robot instance.
	Robot* robot = new Robot();
	//camera 
	Camera* myCam = robot->getCamera("camera");
	myCam->enable(TIME_STEP);

	//lidar
	Lidar* myPuck = robot->getLidar("Velodyne Puck");
	myPuck->enable(TIME_STEP);
	myPuck->enablePointCloud();

	GPS* myGps = robot->getGPS("gps");
	myGps->enable(TIME_STEP);

	InertialUnit* myImu = robot->getInertialUnit("IMU");
	myImu->enable(TIME_STEP);

	Gyro* myGyro = robot->getGyro("gyro");
	myGyro->enable(TIME_STEP);

	//bogies
	Motor* BackLeftBogie = robot->getMotor("BackLeftBogie");
	Motor* FrontLeftBogie = robot->getMotor("FrontLeftBogie");
	Motor* BackRightBogie = robot->getMotor("BackRightBogie");
	Motor* FrontRightBogie = robot->getMotor("FrontRightBogie");

	map<string, Motor*> bogies = {
		{"FrontLeftBogie",FrontLeftBogie},
		{"BackLeftBogie",BackLeftBogie},
		{"FrontRightBogie",FrontRightBogie},
		{"BackRightBogie",BackRightBogie}
	};

	//arms
	Motor* FrontLeftArm = robot->getMotor("FrontLeftArm");
	Motor* BackLeftArm = robot->getMotor("BackLeftArm");
	Motor* FrontRightArm = robot->getMotor("FrontRightArm");
	Motor* BackRightArm = robot->getMotor("BackRightArm");

	map<string, Motor*> arms = {
		{"FrontLeftArm",FrontLeftArm},
		{"BackLeftArm",BackLeftArm},
		{"FrontRightArm",FrontRightArm},
		{"BackRightArm",BackRightArm}
	};

	//wheels
	Motor* FrontLeftWheel = robot->getMotor("FrontLeftWheel");
	Motor* MiddleLeftWheel = robot->getMotor("MiddleLeftWheel");
	Motor* BackLeftWheel = robot->getMotor("BackLeftWheel");
	Motor* FrontRightWheel = robot->getMotor("FrontRightWheel");
	Motor* MiddleRightWheel = robot->getMotor("MiddleRightWheel");
	Motor* BackRightWheel = robot->getMotor("BackRightWheel");
	FrontLeftWheel->setPosition(numeric_limits<double>::infinity());
	MiddleLeftWheel->setPosition(numeric_limits<double>::infinity());
	BackLeftWheel->setPosition(numeric_limits<double>::infinity());
	FrontRightWheel->setPosition(numeric_limits<double>::infinity());
	MiddleRightWheel->setPosition(numeric_limits<double>::infinity());
	BackRightWheel->setPosition(numeric_limits<double>::infinity());

	map<string, Motor*> wheels = {
	{ "FrontLeftWheel",FrontLeftWheel },
	{"MiddleLeftWheel",MiddleLeftWheel },
	{"BackLeftWheel",BackLeftWheel },
	{"FrontRightWheel",FrontRightWheel },
	{"MiddleRightWheel",MiddleRightWheel },
	{"BackRightWheel",BackRightWheel }
	};

	std::cout << "controls: W - forward, Q - left, E - right, S - spin" << endl;
	std::cout << "features: A - Enter auto scan mode, B - stop auto scan mode, M - build map, G - send GPS, O - Save CamView, " << endl;
	move_6_wheels(wheels, 1.0);

	Keyboard* myKeyboard = robot->getKeyboard();
	myKeyboard->enable(TIME_STEP);


	int counter = 0;
	int imageNumber = 0;
	bool au = false;
	bool t_started = false;
	Params params;
	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	while (robot->step(TIME_STEP) != -1) {
		int key = myKeyboard->getKey();
		if (key != -1)
		{
			ofstream controlFile;
			controlFile.open("../../../website/Flask/static/files/controls/controls.txt");
			controlFile << (char)key << endl;
			controlFile.close();
		}
		switch (key) {
		case 'W':
			// forwards
			wheels_straight(arms);
			move_6_wheels(wheels, 1.0);
			break;
		case 'X':
			// backwards
			wheels_straight(arms);
			move_6_wheels(wheels, -1.0);
			break;
		case 'Q':
			// forwards left
			turn_wheels_left(arms);
			move_4_wheels(wheels, 1.0);
			break;
		case 'E':
			// forwards right
			turn_wheels_right(arms);
			move_4_wheels(wheels, 1.0);
			break;
		case 'Y':
			// backwards left
			turn_wheels_left(arms);
			move_4_wheels(wheels, -1.0);
			break;
		case 'C':
			// backwards right
			turn_wheels_right(arms);
			move_4_wheels(wheels, -1.0);
			break;
		case 'S':
			// spin counter-clockwise
			turn_around(arms, wheels, 1.0);
			printf("S is pressed\n");
			break;
		case 'O': //save camera output and post request
		{
			myCam->saveImage("../../../website/Flask/static/pic/cam-wb.jpg", 80);
			ofstream logfile;
			logfile.open("../../../website/Flask/static/files/logs/logs.txt");
			logfile << "Camera live capture updated..." << endl;
			logfile.close();
			break;
		}
		case 'G':
		{
			const double* gps = myGps->getValues();
			string gpsReading = "x: " + to_string(*(gps + 1)) + ", y: " + to_string(*(gps + 2)) + ",z: " + to_string(*gps);
			ofstream logfile;
			logfile.open("../../../website/Flask/static/files/logs/logs.txt");
			logfile << "Sending GPS data..." << endl;
			logfile.close();

			ofstream f;
			f.open("../../../website/Flask/static/files/gps/gps.txt");
			f << gpsReading << endl;
			f.close();
			break;
		}
		case 'A':
			std::cout << "building map automatically" << endl;
			au = true;
			break;
		case 'B':
		std:cout << "stop auto map building" << endl;
			au = false;
			break;
		case 'M':
			std::cout << "generate map file" << endl;
			buildMap(myPuck, myGps, myImu, myGyro);
			break;
		case 'V': //open octovis
		{
			ofstream logfile;
			logfile.open("../../../website/Flask/static/files/logs/logs.txt");
			logfile << "Opening Octovis..." << endl;
			logfile.close();
			system("E:/school/490/CapstoneSimulationWebots/libraries/octomap-1.9.6/bin/octovis.exe E:/school/490/CapstoneSimulationWebots/controllers/tree_controller/webots.bt");
			break;
		}
		}

		if (au)
		{

			if (counter % 4 == 0) //read lidar data every 64ms*4 = 0.256s
			{
				buildMap(myPuck, myGps, myImu, myGyro);
				//buildPath(myPuck, myGps, myImu, myfile, myGyro);
			}

			if (counter % 47 == 0) // send image every 3 seconds
			{
				myCam->saveImage("../../../website/Flask/static/pic/cam-wb.jpg", 80);
				//Params params;
				//params.emplace("uri", "updated");
				//auto res = cli.Post("/cam-wb", params);
				//cout << "request res: " << res->body << endl;
				ofstream logfile;
				logfile.open("../../../website/Flask/static/files/logs/logs.txt");
				logfile << "Camera live capture updated..." << endl;
				logfile.close();
			}

			if (counter % 16 == 0) // send gps every 1 second
			{
				const double* gps = myGps->getValues();
				string gpsReading = "X: " + to_string(*gps) + ", Y: " + to_string(*(gps + 2)) + ",Z: " + to_string(*(gps + 1));
				//params.emplace("gps", gpsReading);
				//cli.Post("/gps-wb", params);
				ofstream logfile;
				logfile.open("../../../website/Flask/static/files/logs/logs.txt");
				logfile << "Sending GPS data..." << endl;
				logfile.close();
				ofstream f;
				f.open("../../../website/Flask/static/files/gps/gps.txt");
				f << gpsReading << endl;
				f.close();
			}

			counter++;
		}


	};

	// Enter here exit cleanup code.
	delete robot;
	return 0;
}
