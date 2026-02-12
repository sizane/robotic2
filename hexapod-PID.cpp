#include <MadgwickAHRS.h>
#include <mpu9250.h>
#include <util/atomic.h>

bfs::Mpu9250 imu(&SPI, 10);
Madgwick MWKfilter;

float yaw, pitch, roll;
float declinationAngle = 7.0 * (57.0 / 60.0) * PI / 180.0; 
unsigned long timer;
int count = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

double setpoint1 = 0; 
double input1 = imu.getPicth; 
double output1 = 0; 
double error1 = 0, lastError1 = 0;
double integral1 = 0, derivative1 = 0;

double setpoint2 = 0; 
double input2 = imu.getRoll; 
double output2 = 0; 
double error2 = 0, lastError2 = 0;
double integral2 = 0, derivative2 = 0;


void setup() {
    Serial.begin(115200);
    imu.setWire(&Wire);

    imu.beginAccel();
    imu.beginGyro();
    imu.beginMag();

    MWkfilter.begin(25); 
    
    timer = millis();
}

void loop(){
    unsigned long currentTime = millis();

    //Update Sensor (Selalu jalankan ini agar filter Madgwick stabil)
    mySensor.accelUpdate();
    mySensor.gyroUpdate();
    mySensor.magUpdate();

    ax = mySensor.accelX(); ay = mySensor.accelY(); az = mySensor.accelZ();
    gx = mySensor.gyroX();  gy = mySensor.gyroY();  gz = mySensor.gyroZ();
    mx = mySensor.magX();   my = mySensor.magY();   mz = mySensor.magZ();

    MadgwickFilter.update(gx, gy, gz, ax, ay, az, mx, my, mz);  

    //hitung pitch dan roll
    pitch = MWKfilter.getPitch(); // Untuk PID Depan-Belakang
    roll  = MWKfilter.getRoll();
}