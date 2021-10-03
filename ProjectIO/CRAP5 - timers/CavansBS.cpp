// #include <xArmServoController.h>
// #include <arduino.h>
// #define PI 3.141592653
// #define use_sim_numbers true
// #include <SoftwareSerial.h>

// // To use SoftwareSerial:
// // 1. Uncomment include statement above and following block.
// // 2. Update xArmServoController with mySerial.
// // 3. Change Serial1.begin to mySerial.begin.
 
// #define rxPin D6
// #define txPin D7
// SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

// xArmServoController myArm = xArmServoController(xArm, mySerial);

// void sam (double* angles){
// 	xArmServo servos[] = {{1, 500},
//                         {2, 500},
//                         {3, 500},
//                         {4, 500},
//                         {5, 500},
//                         {6, 500}};
// 	myArm.getPosition(servos, 6);
// 	angles[0] = (((servos[2-1].position)*PI)/792)-(PI/2); // change to be -pi/2 to pi/2 range
// 	angles[1] = ((((servos[6-1].position)-113)*PI)/753)-(PI/2);
// 	angles[2] = ((((servos[5-1].position)-112)*PI)/762)-(PI/2);
// 	angles[3] = ((((servos[4-1].position)-132)*(PI)/745)-(PI/2));
// 	angles[4] = ((servos[3-1].position)*PI)/792;

// 	if(use_sim_numbers) angles[2] += PI/2;
// }


// void setup() {
// 	Serial.begin(9600);
//   mySerial.begin(9600);

//  // xArm servo positions
//    xArmServo home[] = {{1, 150},
// 					  {2, 500},
// 					  {3, 500},
// 					  {4, 500},
// 					  {5, 500},
// 					  {6, 500}};
//   xArmServo bow[] = {{1, 650},
// 					 {6, 130},
// 					 {4, 845},
// 					 {5, 650}};
 
// //   myArm.setPosition(home, 6, 1000, true);
// //   delay(1000);
// //   myArm.setPosition(bow, 4, 3000, true);
// //   delay(1000);
// //   myArm.setPosition(home, 6, 1000, true);
//   delay(3000);

//   double Store[51][5] = {{0.54042, 0.33443, 0.65867, 0.60096, 0},
// 						  {0.57791, 0.38313, 0.63564, 0.55312, 0},
// 						  {0.61633, 0.4257, 0.61461, 0.51147, 0},
// 						  {0.65545, 0.46107, 0.59647, 0.477, 0},
// 						  {0.69511, 0.48817, 0.58217, 0.45072, 0},
// 						  {0.73512, 0.506, 0.57256, 0.43351, 0},
// 						  {0.77533, 0.51379, 0.5683, 0.42602, 0},
// 						  {0.81558, 0.51118, 0.56973, 0.42852, 0},
// 						  {0.85571, 0.49829, 0.57678, 0.4409, 0},
// 						  {0.89557, 0.47567, 0.58896, 0.46262, 0},
// 						  {0.93498, 0.44421, 0.60554, 0.4929, 0},
// 						  {0.97378, 0.40497, 0.6256, 0.53077, 0},
// 						  {1.0118, 0.35902, 0.64821, 0.57524, 0},
// 						  {1.0487, 0.30738, 0.6725, 0.62535, 0},
// 						  {1.0844, 0.25094, 0.69767, 0.68021, 0},
// 						  {1.1185, 0.19044, 0.72305, 0.73906, 0},
// 						  {1.1507, 0.12655, 0.74802, 0.80118, 0},
// 						  {1.1806, 0.059795, 0.77208, 0.86594, 0},
// 						  {1.2077, -0.0093251, 0.79478, 0.93275, 0},
// 						  {1.2315, -0.080369, 0.8157, 1.001, 0},
// 						  {1.2512, -0.1529, 0.8345, 1.0701, 0},
// 						  {1.2659, -0.22645, 0.85087, 1.1395, 0},
// 						  {1.2745, -0.30048, 0.86452, 1.2083, 0},
// 						  {1.2758, -0.37427, 0.8752, 1.2758, 0},
// 						  {1.2679, -0.44695, 0.88274, 1.3412, 0},
// 						  {1.249, -0.51738, 0.88715, 1.4033, 0},
// 						  {1.2169, -0.58406, 0.88864, 1.4607, 0},
// 						  {1.1692, -0.64508, 0.88766, 1.512, 0},
// 						  {1.1041, -0.69808, 0.88499, 1.5554, 0},
// 						  {1.0212, -0.7403, 0.88163, 1.5892, 0},
// 						  {0.92253, -0.76894, 0.87872, 1.6117, 0},
// 						  {0.81339, -0.78168, 0.87725, 1.6216, 0},
// 						  {0.702, -0.7774, 0.87776, 1.6182, 0},
// 						  {0.5972, -0.75652, 0.88011, 1.6019, 0},
// 						  {0.50595, -0.72087, 0.88358, 1.5734, 0},
// 						  {0.43189, -0.6731, 0.88706, 1.5344, 0},
// 						  {0.37568, -0.61604, 0.88947, 1.4865, 0},
// 						  {0.33602, -0.55225, 0.88991, 1.4315, 0},
// 						  {0.31079, -0.48389, 0.88777, 1.371, 0},
// 						  {0.2977, -0.41262, 0.8827, 1.3065, 0},
// 						  {0.29462, -0.33972, 0.87457, 1.2392, 0},
// 						  {0.29974, -0.26615, 0.86339, 1.1701, 0},
// 						  {0.31158, -0.19265, 0.84926, 1.1001, 0},
// 						  {0.3289, -0.1198, 0.8324, 1.0302, 0},
// 						  {0.35072, -0.048122, 0.81313, 0.96082, 0},
// 						  {0.37625, 0.021903, 0.79181, 0.89268, 0},
// 						  {0.40483, 0.089823, 0.76885, 0.82633, 0},
// 						  {0.43594, 0.15517, 0.74469, 0.76234, 0},
// 						  {0.46915, 0.21742, 0.71981, 0.70133, 0},
// 						  {0.50408, 0.27598, 0.69477, 0.64397, 0},
// 						  {0.54042, 0.33015, 0.67019, 0.59102, 0}};
		
// 		// double desired[5] = {Store[1][0],Store[1][1],Store[2][2],Store[1][3], Store[1][4]};
		
// 		// // testing

// 		// double desired[5] = {0,0,0,0,0};
// 		// for(int i=0; i<51;i++){
// 		// 	desired[0] = PI*i/50;
			
// 		// 	xArmServo destination[5];
// 		// 	destination[0] = {2,  (unsigned int)((desired[0]+(PI/2))*792/PI)+0};//{2, (unsigned int)(22*i)};
// 		// 	destination[1] = {6, (unsigned int)(((desired[1]+(PI/2))*778/PI)+93)};//{6, 500};
// 		// 	destination[2] = {5, (unsigned int)(((desired[2]+(PI/2))*733)/(PI)+110)};//{5, 500};
// 		// 	destination[3] = {4, (unsigned int)((desired[3]+(PI/2))*745/(PI)+132)};//{4, 500};
// 		// 	destination[4] = {3, (unsigned int)((desired[4]+PI)*1140/(1.5*PI))};//{3, 500};
// 		// 	Serial.print(i, DEC);
// 		// 	myArm.setPosition(destination, 5, 200, true);


// 		// }


// 		// if(use_sim_numbers){
// 		// 	//desired[1] += PI/2;
// 		// 	desired[2] -= PI/2;
// 		// }
		


// 		// myArm.setPosition(destination, 5, 3000, true);

// 	for (int i = 0; i < 15; i++) {

// 		Serial.print(i, DEC);
// 		double desired[5] = {Store[i][0],Store[i][1],Store[i][2],Store[i][3], Store[i][4]};
		
// 		if(use_sim_numbers){
// 			//desired[1] += PI/2;
// 			desired[2] -= PI/2;
// 		}
		
		
// 		xArmServo destination[6];
// 		destination[0] = {2,  (unsigned int)((desired[0]+(PI/2))*792/PI)+0};
// 		destination[1] = {6, (unsigned int)(((desired[1]+(PI/2))*753/PI)+113)};
// 		destination[2] = {5, (unsigned int)((desired[2]+(PI/2))*762/(PI)+112)};
// 		destination[3] = {4, (unsigned int)((desired[3]+(PI/2))*745/(PI)+132)};
// 		destination[4] = {3, (unsigned int)((desired[4]+(PI/2))*1000/(PI))};
// 		destination[5] = {1, 150};
// 		int x;
// 		if(i<3) x =1000/i;
// 		else x = 150;
// 		myArm.setPosition(destination, 5, x, true);
// 		// delay(100);



        
// 		xArmServo servos[] = {{1, 500},
//                         {2, 500},
//                         {3, 500},
//                         {4, 500},
//                         {5, 500},
//                         {6, 500}};

//   double angles[] = {1,2,3,4,5};
//   sam(angles);
//   Serial.println();
  
// //   for (int k = 0; k < 5; k++) {
// //       Serial.print("\nServo ");
// //       Serial.print(k+1, DEC);
// //       Serial.print(" angle: ");
// //       Serial.print(angles[k]);
// //   	}
// 	// Serial.println();

// 	for (int j = 0; j < 5; j++) {

// 	  Serial.print("\nServo ");
//       Serial.print(j+1, DEC);
//       Serial.print(" angle: ");
//       Serial.print(angles[j]);
	
//       Serial.print("\nServo ");
//       Serial.print(j+1, DEC);
//       Serial.print(" error: ");
//       Serial.print(angles[j]-Store[i][j]);
//   	}
// 	Serial.println();
	

// // xArmServo servos[] = {{1, 500},
// //                         {2, 500},
// //                         {3, 500},
// //                         {4, 500},
// //                         {5, 500},
// //                         {6, 500}};

// //   //myArm.servoOff(); // turns off all servo motors.
// //   myArm.getPosition(servos, 6);
// //   Serial.println();
  
// //   for (int i = 0; i < 6; i++) {
// //       Serial.print("\nServo ");
// //       Serial.print(i+1, DEC);
// //       Serial.print(" position: ");
// //       Serial.print(servos[i].position);
// //   	}
// // 	Serial.println();
// }

//   // Your setup here.
// }


// void loop() {
// 	// Your code here.
// 	// xArmServo servos[] = {{1, 500},
//     //                     {2, 500},
//     //                     {3, 500},
//     //                     {4, 500},
//     //                     {5, 500},
//     //                     {6, 500}};

  
// 	//   myArm.getPosition(servos, 6);
// 	//   Serial.println();
	
// 	//   for (int i = 0; i < 6; i++) {
// 	//       Serial.print("\nServo ");
// 	//       Serial.print(i+1, DEC);
// 	//       Serial.print(" position: ");
// 	//       Serial.print(servos[i].position);
// 	//   	}
// 	Serial.println();

//   double angles[] = {1,2,3,4,5};
//   sam(angles);
//   Serial.println();

// 	for (int j = 0; j < 5; j++) {

// 	  Serial.print("\nServo ");
//       Serial.print(j+1, DEC);
//       Serial.print(" angle: ");
//       Serial.print(angles[j]);
//   	}

// 	delay(2000);
// }
