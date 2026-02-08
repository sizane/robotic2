#include <string.h>

#define trig_depan 6
#define echo_depan 7
#define trig_kanan 8
#define echo_kanan 9
#define trig_kiri 10
#define echo_kiri 11
#define trig_belakang 12
#define echo_belakang 13


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

float trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0;
    else if (x <= b) return (x - a) / (b - a);
    else return (c - x) / (c - b);
}

jarakSet fuzzifyJarak(long distance) {
    jarakSet ds;
    ds.very_close = trimf(distance, 0, 0, 20);
    ds.close = trimf(distance, 10, 30, 50);
    ds.medium = trimf(distance, 40, 60, 80);
    ds.far = trimf(distance, 70, 100, 100);
    return ds;
}

float fuzzyMin(float a, float b) {
    return (a < b) ? a : b;
}

float fuzzyMax(float a, float b) {
    return (a > b) ? a : b;
}

float fuzzyMax3(float a, float b, float c) {
    return fuzzyMax(a, fuzzyMax(b, c));
}
long readUltrasonic(int trigPin, int echoPin) {
        digitalWrite(trigPin, LOW); // Sensor berhenti mengirim gelombang ultrasonik
        delayMicroseconds(2); 
        digitalWrite(trigPin, HIGH); // Sensor akan mengirim gelombang ultrasonik
        delayMicroseconds(10); // Memberi pulsa HIGH selama 10 mikrodetik ke pin TRIG 
        digitalWrite(trigPin, LOW); // Sensor berhenti mengirim gelombang ultrasonik
        long duration = pulseIn(echoPin, HIGH, 30000); 
        // mengukur lama waktu (dalam mikrodetik) pin ECHO menerima sinyal HIGH
        long distance = duration * 0.034 / 2;
        /**
            - Kecepatan suara di udara ≈ 0,0343 cm/µs
            - selang * 0.0343 → menghitung jarak total tempuh suara (pergi + pulang)
            - Dibagi 2 → mendapatkan jarak sebenarnya ke objek (hanya satu arah)
        **/

        return distance;
}

FuzzyOutputPreference arah(jarakSet depan, jarakSet kanan, jarakSet kiri, jarakSet belakang) {
    float pref_turn_left = 0.0;
    float pref_straight = 0.0;
    float pref_turn_right = 0.0;
    float pref_backward = 0.0;

    // RULE 1: Jalan Lurus (Jika depan jauh)
    // Logika: Jika depan Jauh ATAU Sedang -> Lurus
    pref_straight = fuzzyMax(depan.far, depan.medium);

    // RULE 2: Hindari Kanan (Jika kanan dekat, belok kiri)
    // Logika: Kanan Dekat -> Belok Kiri
    pref_turn_left = fuzzyMax(pref_turn_left, kanan.close);

    // RULE 3: Hindari Kiri (Jika kiri dekat, belok kanan)
    // Logika: Kiri Dekat -> Belok Kanan
    pref_turn_right = fuzzyMax(pref_turn_right, kiri.close);

    // RULE 4: Depan Tertutup (Harus memilih kiri/kanan)
    // Jika depan DEKAT dan Kanan JAUH -> Belok Kanan
    float rule_obs_front_turn_right = fuzzyMin(depan.close, kanan.far);
    pref_turn_right = fuzzyMax(pref_turn_right, rule_obs_front_turn_right);

    // Jika depan DEKAT dan Kiri JAUH -> Belok Kiri
    float rule_obs_front_turn_left = fuzzyMin(depan.close, kiri.far);
    pref_turn_left = fuzzyMax(pref_turn_left, rule_obs_front_turn_left);

    // RULE 5: Situasi Bahaya (Mundur)
    // Jika Depan Sangat Dekat -> Mundur
    pref_backward = depan.very_close;
    
    // Kembalikan semua keinginan robot
    FuzzyOutputPreference fo;
    fo.turn_left = pref_turn_left;
    fo.straight = pref_straight;
    fo.turn_right = pref_turn_right;
    fo.backward = pref_backward;
    return fo;
}

const int OUTPUT_SIZE = 21;
const float OUT_MIN = -15.0; // Kiri Mentok / Mundur
const float OUT_MAX = 10.0;  // Kanan Mentok

// Bentuk Output Segitiga
float mf_left_out(float x)     { return trimf(x, -10, -10, 0); }
float mf_straight_out(float x) { return trimf(x, -5, 0, 5); }
float mf_right_out(float x)    { return trimf(x, 0, 10, 10); }
float mf_back_out(float x)     { return trimf(x, -15, -15, -10); }

float defuzzifyCOG(FuzzyOutputPreference output) {
    float numerator = 0.0;   
    float denominator = 0.0; 
    for (int i = 0; i < OUTPUT_SIZE; i++) {
        float x = OUT_MIN + i * (OUT_MAX - OUT_MIN) / (OUTPUT_SIZE - 1);
        
        float combined = 0;
        combined = fuzzyMax(combined, fuzzyMin(output.turn_left, mf_left_out(x)));
        combined = fuzzyMax(combined, fuzzyMin(output.straight, mf_straight_out(x)));
        combined = fuzzyMax(combined, fuzzyMin(output.turn_right, mf_right_out(x)));
        combined = fuzzyMax(combined, fuzzyMin(output.backward, mf_back_out(x)));

        numerator += x * combined;
        denominator += combined;
    }

    if (denominator == 0) return 0.0;
    return numerator / denominator;
}

void setup() {
    Serial.begin(9600);
    pinMode(trig_depan, OUTPUT);
    pinMode(echo_depan, INPUT);
    pinMode(trig_kanan, OUTPUT);
    pinMode(echo_kanan, INPUT);
    pinMode(trig_kiri, OUTPUT);
    pinMode(echo_kiri, INPUT);
    pinMode(trig_belakang, OUTPUT);
    pinMode(echo_belakang, INPUT);
}

void loop() {
    // put your main code here, to run repeatedly:
    long d1 = readUltrasonic(trig_depan, echo_depan);
    long d2 = readUltrasonic(trig_kanan, echo_kanan);
    long d3 = readUltrasonic(trig_kiri, echo_kiri);
    long d4 = readUltrasonic(trig_belakang, echo_belakang);

    //fuzzyfikasi
    jarakSet f_depan = fuzzifyJarak(d1);
    jarakSet f_kanan = fuzzifyJarak(d2);
    jarakSet f_kiri = fuzzifyJarak(d3);
    jarakSet f_belakang = fuzzifyJarak(d4);

    FuzzyOutputPreference otak = arah(f_depan, f_kanan, f_kiri, f_belakang);

    //tahap akhir
    float nilai_akhir = defuzzifyCOG(otak);

    String gerakan = "";
    if (nilai_akhir < -11.0)      gerakan = "MUNDUR";
    else if (nilai_akhir < -2.0)  gerakan = "BELOK KIRI";
    else if (nilai_akhir > 2.0)   gerakan = "BELOK KANAN";
    else                          gerakan = "MAJU LURUS";

    Serial.print("D:"); 
    Serial.print(d1);
    Serial.print(" Ki:"); 
    Serial.print(d3);
    Serial.print(" Ka:"); 
    Serial.print(d2);
    Serial.print(" | Score:"); 
    Serial.print(nilai_akhir);
    Serial.print(" -> "); 
    Serial.println(gerakan);

    delay(3000);
}