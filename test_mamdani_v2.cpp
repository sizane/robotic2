#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define depanTrig 32
#define depanEcho 25
#define kananTrig 26
#define kananEcho 27
#define kiriTrig 4
#define kiriEcho 23
#define belakangTrig 18
#define belakangEcho 19

struct jarakSet {
  float dekat_bet;
  float dekat;
  float lumayan;
  float adohmen;
}

struct FuzOutPre {
  float kekanan;
  float kedepan;
  float kekiri;
  float kebelakang;
}
/*
struct trimf(float x, float a, float b, float c, ) {
  if (x <= a || x >= c) return 0.0;
  else if (x <= b) return (x - a) / (b - a);
  else return (c - x) / (c - b);
}
*/
struct trimf(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0;
  else if (a <= x <= b) return (x - a)/(b - a);
  else if (b <= x <= c) return 1.0;
  else return (d - x)/(d - c);
}

jarakSet FuzJar(long jrk) {
  jarakSet ds;
  ds.deket_bet = trimf(jrk, 0, 5, 12, 25);
  ds.deket = trimf(jrk, 15, 25, 45, 55);
  ds.lumayan = trimf(jrk, 40, 60, 80, 90);
  ds.adohmen = trimf(jrk, 75, 90, 100, 100);
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
        long jrk = duration * 0.034 / 2;
        /**
            - Kecepatan suara di udara ≈ 0,0343 cm/µs
            - selang * 0.0343 → menghitung jarak total tempuh suara (pergi + pulang)
            - Dibagi 2 → mendapatkan jarak sebenarnya ke objek (hanya satu arah)
        **/

        return jrk;
}

FuzOutPre fuzinf(long d1, long d2, long d3, long d4) {
  jarakSet maju = FuzJar(d1);
  jarakSet kanan = FuzJar(d2);
  jarakSet kiri = FuzJar(d3);
  jarakSet mundur = FuzJar(d4);

  float pref_belok_kiri = 0.0;
  float pref_kedepan = 0.0;
  float pref_belok_kanan = 0.0;
  float pref_kebelakang = 0.0;

  float act_rule1 = fuzzyMin(maju.adohmen,fuzzyMin(kanan.dekat, kiri.dekat)); // depan jauh, kanan dekat dan kiri dekat maka maju
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule1);
  float act_rule2 = fuzzyMin(kanan.adohmen,fuzzyMin(kanan.dekat, kiri.dekat)); // kanan jauh, depan dekat, kiri dekat maka belok kanan
    pref_belok_kanan = fuzzyMax(pref_belok_kanan, act_rule2)
  float act_rule3 = fuzzyMin(kiri.adohmen, fuzzyMin(maju.dekat, kiri.dekat)); // kiri jauh, depan dekat, kanan dekat maka belok kiri
    pref_belok_kiri = fuzzyMax(pref_belok_kiri, act_rule3);
  float act_rule4 = fuzzyMin(maju.deket_bet, fuzzyMin(kanan.deket_bet, kiri.deket_bet)); 
    pref_kebelakang = fuzzyMax(pref_kebelakang, act_rule4);
  float act_rule5 = fuzzyMin(maju.adohmen,fuzzyMin(kiri.lumayan, kanan.lumayan));
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule5);
  float act_rule6 = fuzzyMin(maju.lumayan, fuzzyMin(kiri.lumayan, kanan.lumayan));
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule6);
  float act_rule7 = fuzzyMin(kiri.dekat, fuzzyMin(maju.adohmen, kanan.adohmen));
    pref_belok_kanan = fuzzyMax(pref_belok_kanan, act_rule7);
  float act_rule8 = fuzzyMin(kiri.deket, fuzzyMin(maju.adohmen, kiri.adohmen));
    pref_belok_kiri = fuzzyMax(pref_belok_kiri, act_rule8);
  float act_rule9 = fuzzyMin(maju.dekat, fuzzyMin(kiri.adohmen, kanan.adohmen));
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule9);
  float act_rule10 = fuzzyMin(mundur.deket, maju.adohmen);
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule10);
  float act_rule11 = maju.deket_bet;
    pref_kebelakang = fuzzyMax(pref_kebelakang, act_rule11);
  float act_rule12 = fuzzyMin(fuzzyMin(maju.lumayan, kiri.lumayan), fuzzyMin(kanan.lumayan, mundur.lumayan));
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule12);
  float act_rule13 = fuzzyMin(maju.lumayan, fuzzyMin(kiri.dekat, kanan.dekat));
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule13);
  float act_rule14 = fuzzyMin(kiri.dekat, fuzzyMin(maju.adohmen, kanan.lumayan));
    pref_belok_kanan = fuzzyMax(pref_belok_kanan, act_rule14 * 0.5f);
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule14 * 0.3f);
  float act_rule15 = fuzzyMin(kanan.dekat, fuzzyMin(maju.adohmen, kiri.lumayan));
    pref_belok_kiri = fuzzyMax(pref_belok_kiri, act_rule15 * 0.5f);
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule15 * 0.3f);

  FuzOutPre fo;
  fo.belok_kiri = pref_belok_kiri;
  fo.kedepan = pref_kedepan;
  fo.belok_kanan = pref_belok_kanan;
  fo.kebelakang = pref_kebelakang;

  return fo;
}

const int OUTPUT_UNIVERSE_SIZE = 61;
const float OUTPUT_UNIVERSE_MIN = -100.0f;
const float OUTPUT_UNIVERSE_MAX = 100.0f;

float mf_turn_left_out(float x) {
    return trimf(x, -80, -50, -10);
}
float mf_straight_out(float x) {
    return trimf(x, -30, 0, 30);
}
float mf_turn_right_out(float x) {
    return trimf(x, 10, 50, 80);
}

float mf_backward_out(float x) {
    return trimf(x, -100, -70, -40); 
}


float DefFuzCOG(FuzOutPre output) {
    float numerator = 0.0;
    float denominator = 0.0;

    for (int i = 0; i < OUTPUT_UNIVERSE_SIZE; i++) {
        float x = OUTPUT_UNIVERSE_MIN + i * (OUTPUT_UNIVERSE_MAX - OUTPUT_UNIVERSE_MIN) / (OUTPUT_UNIVERSE_SIZE - 1);

        float combined = fuzzyMax(
            fuzzyMax3(
                fuzzyMin(output.turn_left, mf_turn_left_out(x)),
                fuzzyMin(output.straight, mf_straight_out(x)),
                fuzzyMin(output.turn_right, mf_turn_right_out(x))
            ),
            fuzzyMin(output.backward, mf_backward_out(x)) 
        );

        numerator += x * combined;
        denominator += combined;
    }

    if (denominator == 0) return 0.0;
    return numerator / denominator;
}

long d1, d2, d3, d4;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}