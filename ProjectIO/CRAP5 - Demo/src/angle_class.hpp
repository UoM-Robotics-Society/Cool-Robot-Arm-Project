#include <xArmServoController.h>
#define use_sim_numbers false
#include <arduino.h>
#include <SoftwareSerial.h>


class Robot {
    
    private:

    SoftwareSerial mySerial;
    xArmServoController myArm;
    

    xArmServo servos[6];


    public:
        

        
        Robot(uint16_t txpin, uint16_t rxpin):mySerial(txpin,rxpin),myArm(xArm, mySerial){
            //mySerial.begin(9600);
            for(int i=0;i<6;i++) servos[i] = {i+1, 500};
        }

        void comms_start(){
            mySerial.begin(9600);
        }
        void get_angles (double* angles){

            

            myArm.getPosition(servos, 6);
            angles[0] = (((servos[2-1].position)*PI)/792)-(PI/2); 
            angles[1] = ((((servos[3-1].position)-113)*PI)/753)-(PI/2);
            angles[2] = -((((servos[5-1].position)-112)*PI)/762)-(PI/2);
            angles[3] = ((((servos[4-1].position)-132)*(PI)/745)-(PI/2));
            angles[4] = ((servos[6-1].position)*PI)/792;

            if(use_sim_numbers) angles[2] += PI/2;
        }

        void set_angles (double desired[5], int time){

            if(use_sim_numbers){
                    //desired[1] += PI/2;
                    desired[2] -= PI/2;
                }

            xArmServo destination[5];
            destination[0] = {2,  (unsigned int)((desired[0]+(PI/2))*792/PI)+0};  
            destination[1] = {3, (unsigned int)(((desired[1]+(PI/2))*778/PI)+93)};   
            destination[2] = {5, (unsigned int)((((-desired[2])+(PI/2))*733)/(PI)+110)}; 
            destination[3] = {4, (unsigned int)((desired[3]+(PI/2))*745/(PI)+132)};  
            destination[4] = {6, (unsigned int)((desired[4]+PI)*1140/(1.5*PI))};   

            myArm.setPosition(destination, 5, time, true);
        }

        void get_positions(int* positions){
            
            myArm.getPosition(servos, 6);
            positions[0] = servos[2-1].position;
            positions[1] = servos[3-1].position;
            positions[2] = servos[5-1].position;
            positions[3] = servos[4-1].position;
            positions[4] = servos[6-1].position;
            positions[5] = servos[1-1].position;
        }

        void set_positions(unsigned int* positions, int time){  // base up

            servos[0] = {2, positions[0]};
            servos[1] = {3, positions[1]};
            servos[2] = {5, positions[2]};
            servos[3] = {4, positions[3]};
            servos[4] = {6, positions[4]};
            servos[5] = {1, positions[5]};

            myArm.setPosition(servos, 6, time, true);
        }
    
};
