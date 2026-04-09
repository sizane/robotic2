    #include <util/atomic.h> // Untuk keamanan pembacaan variabel encoder

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
    double Kp = 1.5; 
    double Ki = 0.5; 
    double Kd = 0.1; 

    unsigned long lastTime = 0;
    const double limitPWM = 255.0; // Batas PWM Motor

    void setup() {
        Serial.begin(9600);
        pinMode(pinEncoderA, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pinEncoderA)countEncoder, RISING);
        
        pinMode(pinPWM, OUTPUT);
        pinMode(pinDir, OUTPUT);
        lastTime = millis();
    }

    void loop() {
        // Pre-processing: Hitung selang waktu (dt)
        unsigned long currentTime = millis();
        double dt = (double)(currentTime - lastTime) / 1000.0; // Ubah ke detik

        if (dt >= 0.05) { // Hitung setiap 50ms
            // Baca posisi encoder secara aman
            long currentPos;
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                currentPos = encoderPos;
            }

            // Input adalah selisih posisi (kecepatan)
            input = (currentPos - lastPos); 
            lastPos = currentPos;

            // Hitung PID
            error = setpoint - input; // Hitung error

            // Perhitungan Integration dengan Metode Trapezoidal
            double deltaIntegral = ((error + lastError) / 2.0) * dt;
            integral += deltaIntegral;

            // Jika output sudah mentok, jangan tambah integral lagi
            if (abs(output) >= limitPWM && ((error > 0 && integral > 0) || (error < 0 && integral < 0))) {
                integral -= deltaIntegral; 
            }

            derivative = (error - lastError) / dt; // Hitung derivative

            output = (Kp * error) + (Ki * integral) + (Kd * derivative); // Hitung total Ouput u(t)

            // Kirim ke Driver Motor (L298N dll)
            driveMotor(output);

            lastError = error;
            lastTime = currentTime;

            // Tampilkan di Serial Plotter untuk Tuning
            Serial.print(setpoint); Serial.print(" ");
            Serial.println(input);
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