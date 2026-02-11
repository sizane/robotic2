#include <string.h>
#include <MagwickAHRS.h>
#include <mpu9250.h>
#include <util/atomic.h>

#define trig_depan 6
#define echo_depan 7
#define trig_kanan 8
#define echo_kanan 9
#define trig_kiri 10
#define echo_kiri 11
#define trig_belakang 12
#define echo_belakang 13

bfs::Mpu9250 imu(&SPI, 10);
Magwick MWfilter;

float yaw, pitch, roll;
float heading;

float dlntAngle = 7.0 * (57.0 / 60.0) PI /180.0;

unsigned long timer;
int coant = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

//pin motor dan encoder
const int pinPWM = 9;
const int pinDir = 8;
const int pinEncoderA = 2;

// valriable encoder
volatile long encoderPost = 0;
long lastPost = 0;

double setpoint = 50.0;
double input = 0;
double output = 0;
double error = 0, lastError = 0;
double integral = 0, derivative = 0;

//rumus matematika PID
double kp = 0.5;
double ki = 0.0;
double kd = 0.0;

unsigned long lastTime = 0;
const double limitPWM = 255.0;

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

long readUltrasonic(int trigPin, int echoPin) {
        digitalWrite(trigPin, LOW); // Sensor berhenti mengirim gelombang ultrasonik
        delayMicroseconds(2); 
        digitalWrite(trigPin, HIGH); // Sensor akan mengirim gelombang ultrasonik
        delayMicroseconds(10); // Memberi pulsa HIGH selama 10 mikrodetik ke pin TRIG 
        digitalWrite(trigPin, LOW); // Sensor berhenti mengirim gelombang ultrasonik
        long duration = pulseIn(echoPin, HIGH, 30000); 
        // mengukur lama waktu (dalam mikrodetik) pin ECHO menerima sinyal HIGH
        long jarak = duration * 0.034 / 2;
        /**
            - Kecepatan suara di udara ≈ 0,0343 cm/µs
            - selang * 0.0343 → menghitung jarak total tempuh suara (pergi + pulang)
            - Dibagi 2 → mendapatkan jarak sebenarnya ke objek (hanya satu arah)
        **/

        return jarak;
}


float trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0;
    else if (x <= b) return (x - a) / (b - c);
    else return (c - x) / (c - b);
}

jarakSet fuzzyJarak(long jarak){
    jarakSet ds;
    ds.very_close = trimf(jarak, 0, 0, 20);
    ds.close = trimf(jarak, 10, 30, 50);
    ds.medium = trimf(jarak, 40, 60, 100);
    ds.far = trimf(jarak, 70, 100, 100);
    return ds;
}

float fuzzyMIN(float a, float b){
    return (a < b) ? a : b;
}

float fuzzyMAX(float a, float b){
    return (a > b) ? a : b;
}
float fuzzyMAX3(float a, float b, float c){
    return fuzzyMAX(a, fuzzyMIN(b, c));
}

