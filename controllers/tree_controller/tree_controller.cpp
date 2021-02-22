// All the webots classes are defined in the "webots" namespace


#include <iostream>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Lidar.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <windows.h>
#include <map>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "Serial.h"

#define TIME_STEP 64
#define VELOCITY 8.33

using namespace webots;
using namespace std;
using namespace octomap;

void move_4_wheels(map<string, Motor*> wheels, double v) {
    wheels["FrontLeftWheel"]->setVelocity( v * VELOCITY);
    wheels["MiddleLeftWheel"]->setVelocity( v * VELOCITY);
    wheels["BackLeftWheel"]->setVelocity(v * VELOCITY);
    wheels["FrontRightWheel"]->setVelocity(v * VELOCITY);
    wheels["MiddleRightWheel"]->setVelocity( v * VELOCITY);
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

    wheels["FrontLeftWheel"]->setVelocity( v * VELOCITY);
    wheels["MiddleLeftWheel"]->setVelocity( v * VELOCITY);
    wheels["BackLeftWheel"]->setVelocity(v * VELOCITY);
    wheels["FrontRightWheel"]->setVelocity(-v * VELOCITY);
    wheels["MiddleRightWheel"]->setVelocity( -v * VELOCITY);
    wheels["BackRightWheel"]->setVelocity(-v * VELOCITY);

    wheels["MiddleLeftWheel"]->setAvailableTorque(0.0);
    wheels["MiddleRightWheel"]->setAvailableTorque(0.0);
}

void SerialStart()
{
    CSerial serial;
    if (serial.Open(3, 153600))
        cout << "Port opened successfully" << endl;
    else
        cout << "Failed to open port!" << endl;;

    if (serial.Open(3, 153600))
    {
        //static char* szMessage[] = "This is test data";
        const char* szMessage = "send trash omg LALALALAL!!!!";
        int nBytesSent = serial.SendData(szMessage, strlen(szMessage));
        cout << "bytes sent: " << nBytesSent << endl;
        //ASSERT(nBytesSent == strlen(szMessage));
    }
    else
        cout << "Failed to open port!" << endl;;


    if (serial.Open(3, 153600))
    {

        char* lpBuffer = new char[1000];


        int nBytesRead = serial.ReadData(lpBuffer, 1000);
        Sleep(100);
        cout << "byte read:" << nBytesRead << endl;
        for (int i = 0; i < 500; i++)
        {
            Sleep(50);
            if (*(lpBuffer + i) == '\0')
                break;
            cout << *(lpBuffer + i);
        }
        delete[]lpBuffer;
    }
    else
        cout << "Failed to open port!" << endl;
}
    static OcTree tree(0.5);
void buildMap(Lidar * myLidar, GPS * myGps, InertialUnit * myImu)
{
    

    Pointcloud pCloud;
    const double* gps = myGps->getValues();
    point3d origin(0,0,0);
    const double* rpy = myImu->getRollPitchYaw();
    pose6d frameOrigin(*gps, *(gps + 1), *(gps + 2),*rpy,*(rpy+1),*(rpy+2));
    
    for (int i = 2; i < 8; i++) {
        const LidarPoint * points = myLidar->getLayerPointCloud(i);
        for (int j = 0; j < myLidar->getHorizontalResolution(); j+=2)
        {
            LidarPoint p = *(points + j);
            float d = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (d > 98)
                continue;
            /*
            if (p.x > 95 || p.x < -95)
                continue;
            if (p.y > 95 || p.y < -95)
                continue;
            if (p.z > 95 || p.z < -95)
                continue;
                */
            //cout << "x:" << p.x << ",y:" << p.y << ",z:" << p.z << endl;
            pCloud.push_back(p.x, p.y, p.z);
        }
    }

    tree.insertPointCloud(pCloud, origin,frameOrigin,-1,true,true);
    tree.updateInnerOccupancy();

    tree.writeBinary("webots.bt");
    cout << "wrote example file webots.bt" << endl << endl;
}
void print_query_info(point3d query, OcTreeNode* node) {
    if (node != NULL) {
        cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    }
    else
        cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}
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

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  //camera 
  Camera *myCam = robot->getCamera("camera");
  myCam->enable(TIME_STEP);

  //lidar
  Lidar* myPuck = robot->getLidar("Velodyne Puck");
  myPuck->enable(TIME_STEP);
  myPuck->enablePointCloud();

  GPS* myGps = robot->getGPS("gps");
  myGps->enable(TIME_STEP);
  
  InertialUnit* myImu = robot->getInertialUnit("IMU");
  myImu->enable(TIME_STEP);


   //bogies
  Motor * BackLeftBogie = robot->getMotor("BackLeftBogie");
  Motor * FrontLeftBogie = robot->getMotor("FrontLeftBogie");
  Motor * BackRightBogie = robot->getMotor("BackRightBogie");
  Motor * FrontRightBogie = robot->getMotor("FrontRightBogie");

  map<string, Motor*> bogies = {
      {"FrontLeftBogie",FrontLeftBogie},
      {"BackLeftBogie",BackLeftBogie},
      {"FrontRightBogie",FrontRightBogie},
      {"BackRightBogie",BackRightBogie}
  };
  
  //arms
  Motor* FrontLeftArm = robot->getMotor("FrontLeftArm");
  Motor* BackLeftArm = robot->getMotor("BackLeftArm");
  Motor * FrontRightArm = robot->getMotor("FrontRightArm");
  Motor * BackRightArm = robot->getMotor("BackRightArm");

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
  Motor * FrontRightWheel = robot->getMotor("FrontRightWheel");
  Motor * MiddleRightWheel = robot->getMotor("MiddleRightWheel");
  Motor * BackRightWheel = robot->getMotor("BackRightWheel");
  FrontLeftWheel->setPosition(numeric_limits<double>::infinity());
  MiddleLeftWheel    -> setPosition(numeric_limits<double>::infinity());
  BackLeftWheel        -> setPosition(numeric_limits<double>::infinity());
  FrontRightWheel    -> setPosition(numeric_limits<double>::infinity());
  MiddleRightWheel -> setPosition(numeric_limits<double>::infinity());
  BackRightWheel     -> setPosition(numeric_limits<double>::infinity());

  map<string, Motor*> wheels = {
  { "FrontLeftWheel",FrontLeftWheel },
  {"MiddleLeftWheel",MiddleLeftWheel },
  {"BackLeftWheel",BackLeftWheel },
  {"FrontRightWheel",FrontRightWheel },
  {"MiddleRightWheel",MiddleRightWheel },
  {"BackRightWheel",BackRightWheel }
  };

  std::cout << myPuck->getHorizontalResolution() << endl;
  std::cout << "enter some number:" << endl;
  move_6_wheels(wheels, 1.0);

  Keyboard* myKeyboard = robot->getKeyboard();
  myKeyboard->enable(TIME_STEP);



  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
      int key = myKeyboard->getKey();
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
      case 'O':
          std::cout << "getting map" << endl;
          getMap();
          break;
      case 'D':
          std::cout << "decompression started" << endl;
          break;
      case 'M':
          std::cout << "generate map file" << endl;
          buildMap(myPuck, myGps,myImu);
          break;
      }
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

