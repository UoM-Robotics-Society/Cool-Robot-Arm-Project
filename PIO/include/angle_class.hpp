#include <xArmServoController.h>
#define use_sim_numbers false
#include <SoftwareSerial.h>
#include <arduino.h>

#include "debug.h"

enum class Servo : int {
    Base = 0,
    Shoulder = 1,
    Elbow = 3,
    Wrist = 2,
    Hand = 4,
    Effector = 5,
};

typedef struct angles {
    double rotate;
    double shoulder;
    double elbow;
    double wrist;
    double hand;
} angles_t;

constexpr unsigned int ROTATE_FACT = 754;    // 1132 / 1.5
constexpr unsigned int SHOULDER_FACT = 778;  // Also 753
constexpr unsigned int ELBOW_FACT = 733;     // Also 762
constexpr unsigned int WRIST_FACT = 745;
constexpr unsigned int HAND_FACT = 1140;  // Also 792

constexpr unsigned int ROTATE_BIAS = 24;
constexpr unsigned int SHOULDER_BIAS = 93;  // Also 113
constexpr unsigned int ELBOW_BIAS = 110;    // Also 112
constexpr unsigned int WRIST_BIAS = 132;
constexpr unsigned int HAND_BIAS = 0;

constexpr double scale_for_get(double value, unsigned int bias,
                               unsigned int factor) {
    return ((value - (double)bias) * PI / (double)factor);
}
constexpr unsigned int scale_for_set(double value, unsigned int bias,
                                     unsigned int factor) {
    return (unsigned int)((value * (double)factor / PI) + (double)bias);
}

class Robot {
   public:
    SoftwareSerial mySerial;
    xArmServoController myArm;

    xArmServo servos[6];

   public:
    Robot(uint16_t txpin, uint16_t rxpin)
        : mySerial(txpin, rxpin), myArm(xArm, mySerial) {
        // mySerial.begin(9600);
        for (int i = 0; i < 6; i++) servos[i] = {i + 1, 500};
    }

    void comms_start() { mySerial.begin(9600); }

    double servo_pos(Servo servo) {
        double value = servos[(int)servo].position;
        if (value > 100000 || value < -100000) {
            print_debug("WARNING: Faking invalid servo reading\n");
            return 0;
        }
        return value;
    }

    void get_angles(angles_t* angles) {
        myArm.getPosition(servos, 6);

        angles->rotate =
            scale_for_get(servo_pos(Servo::Base), ROTATE_BIAS, ROTATE_FACT);
        angles->shoulder = scale_for_get(servo_pos(Servo::Shoulder),
                                         SHOULDER_BIAS, SHOULDER_FACT) -
                           (PI / 2);
        angles->elbow = (-scale_for_get(servo_pos(Servo::Elbow), ELBOW_BIAS,
                                        ELBOW_FACT)) +
                        (PI / 2);
        angles->wrist =
            scale_for_get(servo_pos(Servo::Wrist), WRIST_BIAS, WRIST_FACT) -
            (PI / 2);
        angles->hand =
            scale_for_get(servo_pos(Servo::Wrist), WRIST_BIAS, WRIST_FACT);

        if (use_sim_numbers) angles->elbow += PI / 2;
    }
    void set_angles(angles_t* desired) { set_angles(desired, -1); }
    void set_angles(angles_t* desired, int time) {
        if (use_sim_numbers) desired->elbow -= PI / 2;

        xArmServo destination[5];
        destination[0] = {
            2, scale_for_set(desired->rotate, ROTATE_BIAS, ROTATE_FACT)};
        destination[1] = {3, scale_for_set(desired->shoulder + (PI / 2),
                                           SHOULDER_BIAS, SHOULDER_FACT)};
        destination[2] = {5, scale_for_set((-desired->elbow) + (PI / 2),
                                           ELBOW_BIAS, ELBOW_FACT)};
        destination[3] = {4, scale_for_set(desired->wrist + (PI / 2),
                                           WRIST_BIAS, WRIST_FACT)};
        destination[4] = {6,
                          scale_for_set(desired->hand, HAND_BIAS, HAND_FACT)};

        if (time >= 0)
            myArm.setPosition(destination, 5, time, true);
        else
            myArm.setPosition(destination, 5, 0, false);

        // Undo changes
        if (use_sim_numbers) desired->elbow += PI / 2;
    }

    void get_positions(int* positions) {
        myArm.getPosition(servos, 6);
        positions[0] = servos[2 - 1].position;
        positions[1] = servos[3 - 1].position;
        positions[2] = servos[5 - 1].position;
        positions[3] = servos[4 - 1].position;
        positions[4] = servos[6 - 1].position;
        positions[5] = servos[1 - 1].position;
    }

    void set_positions(unsigned int* positions, int time) {  // base up

        servos[0] = {2, positions[0]};
        servos[1] = {3, positions[1]};
        servos[2] = {5, positions[2]};
        servos[3] = {4, positions[3]};
        servos[4] = {6, positions[4]};
        servos[5] = {1, positions[5]};

        myArm.setPosition(servos, 6, time, true);
    }

    void stop() { myArm.servoOff(); }
    void beep() { myArm.beep(); }
    void home() {
        angles_t angles = {0, 0, 0, 0, 0};
        set_angles(&angles, 1000);
    }
};
