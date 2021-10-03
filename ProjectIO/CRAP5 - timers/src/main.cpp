#include <xArmServoController.h>
#include <arduino.h>
#include <SoftwareSerial.h>
#include <angle_class.hpp>

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <string>
#include <sstream>

#include "la.h"
#include "linesearch.h"
#include "forwardkinematics.h"
#include "BFGS.h"

#define GOAL_AXIS_MAX 0.3
#define GOAL_AXIS_MIN 0.1
#define DEBUG_START_INFO true


//object setup

#define rx D6
#define tx D7

Robot CRAP (tx, rx);

LineSearch ls(0.057,0.365,0.430,0);
ForwardKinematics fktoo;

void setup(){
    Serial.begin(9600);
    CRAP.comms_start();
    delay(3000);
    Serial.println("Working!");

    
    double angles[5];
    CRAP.get_angles(angles);
	Serial.println("Got angles!");
    LA::vecd<5> start = angles; 
	//Serial.println(std::string("Motor Angles: " + std::to_string(start[0]) + " " + std::to_string(start[1]) + " " + std::to_string(start[2]) + " " + std::to_string(start[3]) + " " + std::to_string(start[4])).c_str());
	LA::vecd<3> pos = fktoo.GetExtendedPositionVector(start);
	//Serial.println(std::string("Arm position: " + std::to_string(pos[0]) + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2])).c_str());
	
    double x= 0.08, y= 0.08, z = 0.2;


    LA::vecd<3> position;
    ls.set_goal(x, y, z);
	double testcost= ls.cost_function(start, 0, MU_INIT);
	//Serial.println(std::string("Cost: " + std::to_string(testcost)).c_str());
    position = { x, y, z };
    bool goalInBounds = ls.InBoundsPos(position);
    
    if(!goalInBounds) 
		Serial.print("Not in bounds");
    else 
    {
		Serial.println("\nBFGS Engaged!");
        BFGS bfgs = BFGS();
        bfgs.debugIteration = true;
        LA::vecd<5> end;
        bool reachedGoal = bfgs.Run(x, y, z, start, end);
        
        bool isResultInBounds = ls.InBounds(end);
        if (!isResultInBounds) 
            Serial.println("Result not in Bounds");
        else
        {
			Serial.println("Moving arm ");
            for (int i = 0; i<5; i++) 
                angles[i] = end[i];
            CRAP.set_angles(angles, 2000);
        }
		//Serial.println("---------------------");
		//Serial.println("Caluclated Motor Angles: ");
		// std::stringstream ss;
		// std::string str;
		//Serial.println("---------------------");
		//Serial.println("Caluclated Motor Angles: ");
		
		// ss << end[0] << " " << end[1] << " " << end[2] << " " << end[3] << " " << end[4] << std::endl;
		// str = ss.str();
		//Serial.println(str.c_str());
		// ss.clear(); 
		isResultInBounds = ls.InBounds(end);
		Serial.print("Result in Bounds: "); 
		Serial.println(isResultInBounds);
		
		ls.set_goal(x,y,z);
		double cost = ls.cost_function(end, 0, 0.00001);
		Serial.print("Final Cost = "); 
		Serial.println(cost);

		//std::cout << "Final Grad = " << std::endl;
		//LA::print(ls.cost_function_gradient(end, 0.0, 0.00001));
		
		LA::vecd<3> pos = fktoo.GetExtendedPositionVector(end);
		// Serial.println("---------------------");
		// ss << "Target Position: " << x << " " << y << " " << z << std::endl;
		// str = ss.str();
		// Serial.println(str.c_str());
		// ss.clear();

		Serial.println("Where Fk thinks we are: ");
		Serial.println(pos[0], DEC);
		Serial.println(pos[1], DEC);
		Serial.println(pos[2], DEC);

		// ss << "FK Calculated Position Based off Angles: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
		// str = ss.str();
		// Serial.println(str.c_str());
		// Serial.println("---------------------");
	}
}


void loop(){

   
}