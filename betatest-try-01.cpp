#include <string.h>
#include <MagwickAHRS.h>
#include <MPU9250_asukiaaa.h>
#include <util/atomic.h>

#define trig_depan 6
#define echo_depan 7
#define trig_kanan 8
#define echo_kanan 9
#define trig_kiri 10
#define echo_kiri 11
#define trig_belakang 12
#define echo_belakang 13

MPU9250_asukiaaa mpuSensor;
Magwick MWfilter;

float yaw, pitch, roll;
float heading;

float dlntAngle = 7.0 * (57.0 / 60.0) PI /180.0;

unsigned long timer;
int coant = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

struct jarakSet {
    float very_close;
    float close;
    float medium;
    float far;
};

struct FuzzyOutputPreference {
    float turn_left;
    float straight;
    float turn_right;
    float backward; 
};

