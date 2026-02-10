#include <MadgwickAHRS.h>
#include <MPU9250_asukiaaa.h>
#include <util/atomic.h>

MPU9250_asukiaaa mySensor;
Madgwick MadgwickFilter;


float yaw, pitch, roll;
float heading;

float declinationAngle = 7.0 * (57.0 / 60.0) * PI / 180.0; 

unsigned long timer;
int count = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

// Pin Motor & Encoder
const int pinPWM = 9;
const int pinDir = 8;
const int pinEncoderA = 2; // Pin interrupt

// Variabel Encoder
volatile long encoderPos = 0; 
long lastPos = 0;

// Variabel PID
double setpoint = 50.0; // Target: 50 pulsa per interval
double input = 0; // Kecepatan aktual dari sensor
double output = 0; // Hasil perhitungan PID untuk PWM
double error = 0, lastError = 0;
double integral = 0, derivative = 0;

// Konstanta PID (Mulai dari Kp saja dulu saat tuning)
double Kp = 0.5; 
double Ki = 0.0; 
double Kd = 0.0; 

unsigned long lastTime = 0;
const double limitPWM = 255.0; // Batas PWM Motor


void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(pinEncoderA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinEncoderA), countEncoder, RISING);
    
    pinMode(pinPWM, OUTPUT);
    pinMode(pinDir, OUTPUT);
    lastTime = millis();

    mySensor.setWire(&Wire);

    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();

    MadgwickFilter.begin(25); 
    
    timer = millis();
}

void loop() {
    unsigned long currentTime = millis();

    // 1. Update Sensor (Selalu jalankan ini agar filter Madgwick stabil)
    mySensor.accelUpdate();
    mySensor.gyroUpdate();
    mySensor.magUpdate();

    ax = mySensor.accelX(); ay = mySensor.accelY(); az = mySensor.accelZ();
    gx = mySensor.gyroX();  gy = mySensor.gyroY();  gz = mySensor.gyroZ();
    mx = mySensor.magX();   my = mySensor.magY();   mz = mySensor.magZ();

    MadgwickFilter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // 2. Hitung Heading
    heading = atan2(-my, -mx) + declinationAngle;
    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;

    float headingDegrees = heading * 180.0 / PI;

    // 3. Jalankan PID setiap 50ms
    if (currentTime - lastTime >= 50) {
        double dt = (double)(currentTime - lastTime) / 1000.0;
        
        input = headingDegrees; 
        error = setpoint - input;

        // Logika Jarak Terpendek
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        // Deadzone agar motor tidak bergetar di titik setpoint
        if (abs(error) < 1.5) {
            output = 0;
            integral = 0;
        } else {
            double deltaIntegral = ((error + lastError) / 2.0) * dt;
            integral += deltaIntegral;

            // Anti-Windup
            if (abs(output) >= limitPWM && ((error > 0 && integral > 0) || (error < 0 && integral < 0))) {
                integral -= deltaIntegral; 
            }

            derivative = (error - lastError) / dt;
            output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        }

        driveMotor(output);

        // 4. Output ke Serial Plotter (Format: Setpoint, Input, Output_PWM)
        Serial.print("Target:"); Serial.print(setpoint); Serial.print(",");
        Serial.print("Posisi:"); Serial.print(input); Serial.print(",");
        Serial.print("PWM:"); Serial.println(output);

        lastError = error;
        lastTime = currentTime;
    }
}

void countEncoder() {
    encoderPos++;
}

void driveMotor(double pwm) {
    int speed = constrain(abs(pwm), 0, 255);
    digitalWrite(pinDir, pwm > 0 ? HIGH : LOW);
    analogWrite(pinPWM, speed);
}