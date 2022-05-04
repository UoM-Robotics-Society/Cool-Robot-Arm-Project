#include <SoftwareSerial.h>
#include <arduino.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>
#include <string>

#include "BFGS.h"
#include "angle_class.hpp"
#include "debug.h"
#include "forwardkinematics.h"
#include "la.h"
#include "linesearch.h"
#include "parabola_array.h"
#include "xArmServoController.h"

#define GOAL_AXIS_MAX 0.3
#define GOAL_AXIS_MIN 0.1
#define DEBUG_START_INFO true

#define rx D6
#define tx D7

Robot CRAP(tx, rx);
BFGS bfgs();

// Old consants
// LineSearch ls(0.057, 0.365, 0.430, -0.08);
LineSearch ls(0.057, 0.7, 0.430, -0.08);
ForwardKinematics fktoo;

void get_random_start(angles_t* output) {
    output->rotate = LA::Pi * (((float)rand() / RAND_MAX) - 0.5);
    output->shoulder = LA::Pi * (((float)rand() / RAND_MAX) - 0.5);
    output->elbow = LA::Pi * (((float)rand() / RAND_MAX) - 0.5);
    output->wrist = LA::Pi * (((float)rand() / RAND_MAX) - 0.5);
    output->hand = LA::Pi * (((float)rand() / RAND_MAX) - 0.5);
}

void vprint_debug(const char* fmt, va_list args) {
    int size = vsnprintf(NULL, 0, fmt, args);
    char* a = (char*)malloc(size + 1);
    vsprintf(a, fmt, args);

    Serial.write('\xff');
    Serial.write(size);
    Serial.print(a);

    free(a);
}
void print_debug(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprint_debug(fmt, args);
    va_end(args);
}

__inline double bezier(double p0, double p1, double p2, double t) {
    return (p0 * t * t) + (p1 * 2 * t * (1 - t)) + (p2 * (1 - t) * (1 - t));
}
__inline double bezier_c(double p0, double pc, double p2, double t) {
    double p1 = (2 * pc) - (p0 / 2) - (p2 / 2);
    return bezier(p0, p1, p2, t);
}
__inline angles_t bezier_angles(angles_t p0, angles_t pc, angles_t p2,
                                double t) {
    angles_t out;
    out.rotate = bezier_c(p0.rotate, pc.rotate, p2.rotate, t);
    out.shoulder = bezier_c(p0.shoulder, pc.shoulder, p2.shoulder, t);
    out.elbow = bezier_c(p0.elbow, pc.elbow, p2.elbow, t);
    out.wrist = bezier_c(p0.wrist, pc.wrist, p2.wrist, t);
    out.hand = bezier_c(p0.hand, pc.hand, p2.hand, t);
    return out;
}
__inline double lerp(double from, double to, double amount) {
    return from + (to - from) * amount;
}
__inline angles_t lerp_angles(angles_t from, angles_t to, double amount) {
    angles_t out;
    out.rotate = lerp(from.rotate, to.rotate, amount);
    out.shoulder = lerp(from.shoulder, to.shoulder, amount);
    out.elbow = lerp(from.elbow, to.elbow, amount);
    out.wrist = lerp(from.wrist, to.wrist, amount);
    out.hand = lerp(from.hand, to.hand, amount);
    return out;
}
__inline double lerp3(double from, double to1, double to2, double t1,
                      double t2) {
    return from + (to1 - from) * t1 + (to2 - to1) * t2;
}
__inline angles_t lerp3_angles(angles_t from, angles_t to1, angles_t to2,
                               double t1, double t2) {
    angles_t out;
    out.rotate = lerp3(from.rotate, to1.rotate, to2.rotate, t1, t2);
    out.shoulder = lerp3(from.shoulder, to1.shoulder, to2.shoulder, t1, t2);
    out.elbow = lerp3(from.elbow, to1.elbow, to2.elbow, t1, t2);
    out.wrist = lerp3(from.wrist, to1.wrist, to2.wrist, t1, t2);
    out.hand = lerp3(from.hand, to1.hand, to2.hand, t1, t2);
    return out;
}
__inline LA::vecd<3> lerp3_vec(LA::vecd<3> from, LA::vecd<3> to1,
                               LA::vecd<3> to2, double t1, double t2) {
    LA::vecd<3> out;
    out[0] = lerp3(from[0], to1[0], to2[0], t1, t2);
    out[1] = lerp3(from[1], to1[1], to2[1], t1, t2);
    out[2] = lerp3(from[2], to1[2], to2[2], t1, t2);
    return out;
}

__inline LA::vecd<5> angles_to_vec(angles_t angles) {
    return {angles.rotate, angles.shoulder, angles.elbow, angles.wrist,
            angles.hand};
}
__inline void vec_to_angles(LA::vecd<5> vec, angles_t* angles) {
    angles->rotate = vec[0];
    angles->shoulder = vec[1];
    angles->elbow = vec[2];
    angles->wrist = vec[3];
    angles->hand = vec[4];
}

void log_angle(angles_t angles) {
    print_debug("r:%.5f s:%.5f e:%.5f w:%.5f h:%.5f\n", angles.rotate,
                angles.shoulder, angles.elbow, angles.wrist, angles.hand);
}

bool update_goal(double goal[3]) {
    angles_t angles;

    ls.set_goal(goal[0], goal[1], goal[2]);
    if (!ls.InBoundsPos({goal[0], goal[1], goal[2]})) return false;

    BFGS bfgs = BFGS();
    LA::vecd<5> end;
    angles_t start = {0, 0, 0, 0, 0};
    // Try multiple times with random starting conditions
    for (int i = 0; i < 5; i++) {
        if (bfgs.Run(goal[0], goal[1], goal[2], angles_to_vec(start), end))
            goto bfgs_success;
        get_random_start(&start);
    }
    return false;
bfgs_success:

    if (!ls.InBounds(end, true)) return false;

    vec_to_angles(end, &angles);
    // ! NOTE: Sync call. Can be updated to async later.
    CRAP.set_angles(&angles, 500);

    return true;
}

constexpr size_t TAKE_STEPS = 31;
constexpr size_t MOVE_STEPS = 19;
size_t current_step = 0;
double piece_start[3];
double piece_end[3];
bool taking = false;

std::tuple<double, double, double> board_to_world(int x, int y) {
    // TODO: This!
    return std::make_tuple((double)x / 50, (double)y / 50, 0.05);
}

bool perform_move(int fromX, int fromY, int toX, int toY) {
    auto from = board_to_world(fromX, fromY);
    auto to = board_to_world(toX, toY);

    piece_start[0] = std::get<0>(from);
    piece_start[1] = std::get<1>(from);
    piece_start[2] = std::get<2>(from);
    piece_end[0] = std::get<0>(to);
    piece_end[1] = std::get<1>(to);
    piece_end[2] = std::get<2>(to);

    if (update_goal(piece_start)) {
        current_step = 0;
        return true;
    };
    current_step = 99;
    return false;
}

// void calibrate() {
//     // LA::vecd<3> corners[3];
//     angles_t corners_a[6];

//     for (int n = 1; n <= 6; n++) {
//         CRAP.stop();
//         print_debug("Move to position %d\n", n);
//         CRAP.beep();
//         delay(1000);
//         print_debug("3...\n");
//         CRAP.beep();
//         delay(1000);
//         print_debug("2...\n");
//         CRAP.beep();
//         delay(1000);
//         print_debug("1...\n");
//         CRAP.beep();
//         delay(1000);
//         print_debug("Capturing\n");

//         angles_t angles;
//         CRAP.get_angles(&angles);
//         corners_a[n - 1] = angles;

//         log_angle(angles);
//     }

//     CRAP.beep();
//     delay(500);
//     CRAP.beep();
//     delay(500);

//     delay(2000);

//     // double target[3];
//     // target[0] = corners[0][0];
//     // target[1] = corners[0][1];
//     // target[2] = corners[0][2];

//     constexpr double step = 0.1;
//     double move_time = 100;

//     double x = 0;
//     double y = 0;
//     // while (1) {
//     //     while (x < 1) {
//     //         angles_t target_a =
//     //             bezier_angles(corners_a[0], corners_a[1], corners_a[2],
//     x);
//     //         log_angle(target_a);
//     //         CRAP.set_angles(&target_a);
//     //         delay(move_time);
//     //         x += step;
//     //     }
//     //     while (x > 0) {
//     //         angles_t target_a =
//     //             bezier_angles(corners_a[0], corners_a[1], corners_a[2],
//     x);
//     //         log_angle(target_a);
//     //         CRAP.set_angles(&target_a);
//     //         delay(move_time);
//     //         x -= step;
//     //     }
//     // }

//     while (1) {
//         while (x < 1) {
//             y = 0;
//             while (y < 1) {
//                 angles_t p0 = bezier_angles(corners_a[0], corners_a[1],
//                                             corners_a[2], x);
//                 angles_t p1 = bezier_angles(corners_a[3], corners_a[4],
//                                             corners_a[5], x);
//                 angles_t target_a = lerp_angles(p0, p1, y);
//                 CRAP.set_angles(&target_a, move_time);
//                 y += step;
//                 move_time = 50;
//             }
//             move_time = 500;
//             x += step;
//             // while (y > 0) {
//             //     angles_t target_a =
//             //         lerp3_angles(corners_a[0], corners_a[1],
//             corners_a[2],
//             //         x, y);
//             //     CRAP.set_angles(&target_a, move_time);
//             //     y -= step;
//             // }
//         }
//         move_time = 500;
//         x = 0;
//     }

//     delay(1000000);
// }

void calibrate2() {
    while (1) {
        CRAP.stop();
        angles_t angles;
        CRAP.get_angles(&angles);
        LA::vecd<3> pos =
            fktoo.GetExtendedPositionVector(angles_to_vec(angles));

        // Serial.print(angles.rotate / LA::Pi, 10);
        // Serial.print(",");
        // Serial.print(angles.shoulder / LA::Pi, 10);
        // Serial.print(",");
        // Serial.print(angles.elbow / LA::Pi, 10);
        // Serial.print(",");
        // Serial.print(angles.wrist / LA::Pi, 10);
        // Serial.print(",");
        // Serial.print(angles.hand / LA::Pi, 10);
        // Serial.print(" -> ");
        // Serial.print(pos[0]);
        // Serial.print(",");
        // Serial.print(pos[1]);
        // Serial.print(",");
        // Serial.println(pos[2]);
    }
}

void calibrate3() {
    LA::vecd<3> corners[3];

    for (int n = 1; n <= 3; n++) {
        CRAP.stop();
        print_debug("Move to psotion %d\n", n);
        CRAP.beep();
        delay(1000);
        print_debug("3...\n");
        CRAP.beep();
        delay(1000);
        print_debug("2...\n");
        CRAP.beep();
        delay(1000);
        print_debug("1...\n");
        CRAP.beep();
        delay(1000);
        print_debug("Capturing\n");

        angles_t angles;
        CRAP.get_angles(&angles);
        LA::vecd<3> pos =
            fktoo.GetExtendedPositionVector(angles_to_vec(angles));
        corners[n - 1] = pos;

        print_debug("%.5fm,%.5fm,%.5fm\n", pos[0], pos[1], pos[2]);
    }

    CRAP.beep();
    delay(500);
    CRAP.beep();
    delay(500);

    delay(2000);

    // double target[3];
    // target[0] = corners[0][0];
    // target[1] = corners[0][1];
    // target[2] = corners[0][2];

    double x = 4 / 8;
    double y = 4 / 8;

    double goal[3];
    goal[0] = lerp3(corners[0][0], corners[1][0], corners[2][0], x, y);
    goal[1] = lerp3(corners[0][1], corners[1][1], corners[2][1], x, y);
    goal[2] = lerp3(corners[0][2], corners[1][2], corners[2][2], x, y);
    update_goal(goal);

    CRAP.beep();

    delay(1000000);
}

void selfcheck() {
    CRAP.home();
    angles_t angles = {0, 0, 0, 0, 0};

    // Test each axis
    angles.rotate = -LA::Pi / 2;
    CRAP.set_angles(&angles, 1000);
    angles.rotate = LA::Pi / 2;
    CRAP.set_angles(&angles, 2000);
    angles.rotate = 0;
    CRAP.set_angles(&angles, 1000);

    angles.shoulder = -LA::Pi / 2;
    CRAP.set_angles(&angles, 1000);
    angles.shoulder = LA::Pi / 2;
    CRAP.set_angles(&angles, 2000);
    angles.shoulder = 0;
    CRAP.set_angles(&angles, 1000);

    angles.elbow = -LA::Pi / 2;
    CRAP.set_angles(&angles, 1000);
    angles.elbow = LA::Pi / 2;
    CRAP.set_angles(&angles, 2000);
    angles.elbow = 0;
    CRAP.set_angles(&angles, 1000);

    angles.wrist = -LA::Pi / 2;
    CRAP.set_angles(&angles, 1000);
    angles.wrist = LA::Pi / 2;
    CRAP.set_angles(&angles, 2000);
    angles.wrist = 0;
    CRAP.set_angles(&angles, 1000);

    angles.hand = -LA::Pi / 2;
    CRAP.set_angles(&angles, 1000);
    angles.hand = LA::Pi / 2;
    CRAP.set_angles(&angles, 2000);
    angles.hand = 0;
    CRAP.set_angles(&angles, 1000);

    // Test combined
    angles.rotate = -LA::Pi / 8;
    angles.shoulder = -LA::Pi / 8;
    angles.elbow = -LA::Pi / 8;
    angles.wrist = -LA::Pi / 8;
    CRAP.set_angles(&angles, 1000);

    angles.rotate = LA::Pi / 8;
    angles.shoulder = LA::Pi / 8;
    angles.elbow = LA::Pi / 8;
    angles.wrist = LA::Pi / 8;
    CRAP.set_angles(&angles, 1000);

    angles.rotate = -LA::Pi / 8;
    angles.shoulder = -LA::Pi / 8;
    angles.elbow = LA::Pi / 8;
    angles.wrist = LA::Pi / 8;
    CRAP.set_angles(&angles, 1000);

    angles.rotate = LA::Pi / 8;
    angles.shoulder = LA::Pi / 8;
    angles.elbow = -LA::Pi / 8;
    angles.wrist = -LA::Pi / 8;
    CRAP.set_angles(&angles, 1000);

    CRAP.home();
}

void selfcheck_ik() {
    double target[3] = {0, 0, 0.3};

    if (!update_goal(target)) {
        print_debug("STIK0 failed\n");
        CRAP.beep();
    }
    delay(150);

    target[0] = 0.1;
    target[1] = 0.1;
    if (!update_goal(target)) {
        print_debug("STIK1 failed\n");
        CRAP.beep();
    }
    delay(150);

    target[0] = -0.1;
    target[1] = -0.1;
    if (!update_goal(target)) {
        print_debug("STIK2 failed\n");
        CRAP.beep();
    }
    delay(150);

    target[0] = 0.1;
    target[1] = -0.1;
    target[2] = 0;
    if (!update_goal(target)) {
        print_debug("STIK3 failed\n");
        CRAP.beep();
    }
    delay(150);

    target[0] = -0.1;
    target[1] = 0.1;
    target[2] = 0;
    if (!update_goal(target)) {
        print_debug("STIK4 failed\n");
        CRAP.beep();
    }
    delay(150);

    CRAP.home();
}

// void loop() {
//     // Serial.println(current_step);
//     // if (current_step < (taking ? TAKE_STEPS : MOVE_STEPS)) {
//     //     double next[3];
//     //     Serial.print(piece_start[0]);
//     //     Serial.print(",");
//     //     Serial.print(piece_start[1]);
//     //     Serial.print(",");
//     //     Serial.print(piece_start[2]);
//     //     Serial.print(",  ");
//     //     Serial.print(piece_end[0]);
//     //     Serial.print(",");
//     //     Serial.print(piece_end[1]);
//     //     Serial.print(",");
//     //     Serial.print(piece_end[2]);
//     //     Serial.print(",  ");
//     //     Serial.print(taking);
//     //     Serial.print(",  ");
//     //     Serial.print(current_step);
//     //     Serial.print(",  ");
//     //     for (int ord = 0; ord < 3; ord++) {
//     //         next[ord] =
//     //             move_piece(piece_start, piece_end, taking, ord,
//     //             current_step);
//     //         if (ord == 2) next[ord] /= 5;
//     //         Serial.print(next[ord]);
//     //         Serial.print(",");
//     //         if (isnan(next[ord])) goto no_update;
//     //     }
//     //     update_goal(next);
//     // no_update:
//     //     current_step++;
//     // }
//     // Serial.println(".");

//     delay(1000);
// }

#define OP_HOME '\x01'
#define OP_MOVE '\x02'
#define OP_BEEP '\x03'
#define OP_UNLOCK '\x04'
#define OP_MOVE_ANGLES '\x05'
#define OP_ST '\x40'
#define OP_STIK '\x41'
#define OP_PING '\x7f'
#define OP_READ_ANGLE '\x80'
#define OP_READ_POS '\x81'

#define NACK '\x01'
#define ACK '\x02'

#define NACK_OP '\xff'
#define NACK_SUM '\xfe'
#define NACK_ARG '\xfd'
#define NACK_NOGOAL '\xfc'

#define SerialACK() Serial.write(ACK);
#define SerialNACK(x)   \
    Serial.write(NACK); \
    Serial.write((x));
#define SerialRead(x)           \
    while (!Serial.available()) \
        ;                       \
    x = Serial.read();
#define SerialReadMany(x, len)       \
    while (Serial.available() < len) \
        ;                            \
    Serial.readBytes((uint8_t*)(x), len);
#define SerialFlush(x)               \
    while (Serial.available() < len) \
        ;                            \
    for (int _ = len; _ > 0; _--) Serial.read();

bool noargs(int len, int sum) {
    if (len != 0) {
        SerialNACK(NACK_ARG);
        SerialFlush(len + 1);
        return false;
    }
    SerialRead(sum);
    if (sum != 0) {
        SerialNACK(NACK_SUM);
        return false;
    };
    return true;
}

void loop() {
    while (Serial.available() < 2)
        ;

    SerialRead(char op);
    SerialRead(char len);
    bool read = false;

    char row_from, col_from, row_to, col_to, sum;
    double goal[3];
    angles_t angles;
    LA::vecd<3> pos;
    int speed;

    switch (op) {
        case OP_MOVE:
            if (len != sizeof(double) * 3) {
                SerialNACK(NACK_ARG);
                SerialFlush(len + 1);
                return;
            }
            SerialReadMany(&goal[0], sizeof(double));
            SerialReadMany(&goal[1], sizeof(double));
            SerialReadMany(&goal[2], sizeof(double));
            SerialRead(sum);
            read = true;
            // ! Can't be bothered figuring this out atm
            // if (sum != (((uint32_t*)(&goal))[0] + ((uint32_t*)(&goal))[1] +
            //             ((uint32_t*)(&goal))[2] + ((uint32_t*)(&goal))[3] +
            //             ((uint32_t*)(&goal))[4] + ((uint32_t*)(&goal))[5])
            //             & 0xff) {
            //     SerialNACK(NACK_SUM);
            //     return;
            // }
            // move_piece(row_from, col_from, row_to, col_to);

            if (update_goal(goal)) {
                SerialACK();
            } else {
                SerialNACK(NACK_NOGOAL);
            }
            return;
        case OP_MOVE_ANGLES:
            if (len != sizeof(double) * 5 + sizeof(int)) {
                SerialNACK(NACK_ARG);
                SerialFlush(len + 1);
                return;
            }
            SerialReadMany(&angles.rotate, sizeof(double));
            SerialReadMany(&angles.shoulder, sizeof(double));
            SerialReadMany(&angles.elbow, sizeof(double));
            SerialReadMany(&angles.wrist, sizeof(double));
            SerialReadMany(&angles.hand, sizeof(double));
            SerialReadMany(&speed, sizeof(int));
            SerialRead(sum);
            read = true;
            // ! Can't be bothered figuring this out atm
            // if (sum != (((uint32_t*)(&goal))[0] + ((uint32_t*)(&goal))[1] +
            //             ((uint32_t*)(&goal))[2] + ((uint32_t*)(&goal))[3] +
            //             ((uint32_t*)(&goal))[4] + ((uint32_t*)(&goal))[5])
            //             & 0xff) {
            //     SerialNACK(NACK_SUM);
            //     return;
            // }
            // move_piece(row_from, col_from, row_to, col_to);

            CRAP.set_angles(&angles, speed);
            SerialACK();
            return;
        case OP_HOME:
            if (!noargs(len, sum)) return;
            read = true;

            CRAP.home();
            SerialACK();
            return;
        case OP_BEEP:
            if (!noargs(len, sum)) return;
            read = true;

            CRAP.beep();
            SerialACK();
            return;
        case OP_UNLOCK:
            if (!noargs(len, sum)) return;
            read = true;

            CRAP.stop();
            SerialACK();
            return;

        case OP_ST:
            if (!noargs(len, sum)) return;
            read = true;

            SerialACK();
            selfcheck();
            return;
        case OP_STIK:
            if (!noargs(len, sum)) return;
            read = true;

            SerialACK();
            selfcheck_ik();
            return;

        case OP_PING:
            if (!noargs(len, sum)) return;
            read = true;
            SerialACK();
            return;

        case OP_READ_ANGLE:
            if (!noargs(len, sum)) return;
            read = true;

            CRAP.get_angles(&angles);

            SerialACK();
            Serial.write(sizeof(angles_t));
            Serial.write((uint8_t*)&angles, sizeof(angles_t));
            return;
        case OP_READ_POS:
            if (!noargs(len, sum)) return;
            read = true;

            CRAP.get_angles(&angles);
            pos = fktoo.GetExtendedPositionVector(angles_to_vec(angles));

            SerialACK();
            Serial.write(sizeof(double) * 3);
            Serial.write((uint8_t*)&pos[0], sizeof(double));
            Serial.write((uint8_t*)&pos[1], sizeof(double));
            Serial.write((uint8_t*)&pos[2], sizeof(double));
            return;
    }

    if (!read) {
        char counter = 0;
        for (int i = len; i > 0; i--) {
            while (!Serial.available())
                ;
            counter += Serial.read();
        }
        while (!Serial.available())
            ;
        if (counter != Serial.read()) {
            SerialNACK(NACK_SUM);
            return;
        }
    }
    SerialNACK(NACK_OP);
}

void setup() {
    Serial.begin(9600);
    CRAP.comms_start();
    print_debug("I:Starting\n");
    delay(3000);

    // perform_move(5, 5, 10, 5);

    CRAP.home();
    print_debug("I:Ready\n");

    // calibrate();
    // calibrate3();
}
