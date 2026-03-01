#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define depanTrig 32
#define depanEcho 25
#define kananTrig 26
#define kananEcho 27
#define kiriTrig 4
#define kiriEcho 23
#define belakangTrig 18
#define belakangEcho 19

unsigned long previousMillis_sensor = 0;
unsigned long previousMillis_gait   = 0;

const long interval_sensor = 150;   // baca sensor + fuzzy setiap 150 ms
const long interval_gait   =  40;   // update servo/gait setiap 40 ms


struct jarakSet {
    float dekat_bet;
    float dekat;
    float lumayan;
    float adohmen;
};

struct FuzOutPre {
    float kekanan;
    float kedepan;
    float kekiri;
    float kebelakang;
};

float trimf(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0;
    if (x > a && x < b) return (x - a) / (b - a);
    if (x >= b && x <= c) return 1.0;
    return (d - x) / (d - c);
}

jarakSet FuzJar(long jrk) {
    jarakSet ds;
    ds.dekat_bet = trimf(jrk, 0, 5, 12, 25);
    ds.dekat = trimf(jrk, 15, 25, 45, 55);
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
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    long jrk = duration * 0.034 / 2;
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

    float act_rule1 = fuzzyMin(maju.adohmen, fuzzyMin(kanan.dekat, kiri.dekat));
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule1);

    float act_rule2 = fuzzyMin(kanan.adohmen, fuzzyMin(kanan.dekat, kiri.dekat));
    pref_belok_kanan = fuzzyMax(pref_belok_kanan, act_rule2);

    float act_rule3 = fuzzyMin(kiri.adohmen, fuzzyMin(maju.dekat, kiri.dekat));
    pref_belok_kiri = fuzzyMax(pref_belok_kiri, act_rule3);

    float act_rule4 = fuzzyMin(maju.dekat_bet, fuzzyMin(kanan.dekat_bet, kiri.dekat_bet));
    pref_kebelakang = fuzzyMax(pref_kebelakang, act_rule4);

    float act_rule5 = fuzzyMin(maju.adohmen, fuzzyMin(kiri.lumayan, kanan.lumayan));
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule5);

    float act_rule6 = fuzzyMin(maju.lumayan, fuzzyMin(kiri.lumayan, kanan.lumayan));
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule6);

    float act_rule7 = fuzzyMin(kiri.dekat, fuzzyMin(maju.adohmen, kanan.adohmen));
    pref_belok_kanan = fuzzyMax(pref_belok_kanan, act_rule7);

    float act_rule8 = fuzzyMin(kiri.dekat, fuzzyMin(maju.adohmen, kiri.adohmen));
    pref_belok_kiri = fuzzyMax(pref_belok_kiri, act_rule8);

    float act_rule9 = fuzzyMin(maju.dekat, fuzzyMin(kiri.adohmen, kanan.adohmen));
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule9);

    float act_rule10 = fuzzyMin(mundur.dekat, maju.adohmen);
    pref_kedepan = fuzzyMax(pref_kedepan, act_rule10);

    float act_rule11 = maju.dekat_bet;
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
    fo.kekiri = pref_belok_kiri;
    fo.kedepan = pref_kedepan;
    fo.kekanan = pref_belok_kanan;
    fo.kebelakang = pref_kebelakang;

    return fo;
}

const int OUTPUT_UNIVERSE_SIZE = 61;
const float OUTPUT_UNIVERSE_MIN = -100.0f;
const float OUTPUT_UNIVERSE_MAX = 100.0f;

float mf_turn_left_out(float x) {
    return trimf(x, -80, -60, -40, -10);
}

float mf_straight_out(float x) {
    return trimf(x, -30, -10, 10, 30);
}

float mf_turn_right_out(float x) {
    return trimf(x, 10, 40, 60, 80);
}

float mf_backward_out(float x) {
    return trimf(x, -100, -80, -60, -40);
}

float DefFuzCOG(FuzOutPre output) {
    float numerator = 0.0;
    float denominator = 0.0;

    for (int i = 0; i < OUTPUT_UNIVERSE_SIZE; i++) {
        float x = OUTPUT_UNIVERSE_MIN + i * (OUTPUT_UNIVERSE_MAX - OUTPUT_UNIVERSE_MIN) / (OUTPUT_UNIVERSE_SIZE - 1);
        float combined = fuzzyMax(
            fuzzyMax3(
                fuzzyMin(output.kekiri, mf_turn_left_out(x)),
                fuzzyMin(output.kedepan, mf_straight_out(x)),
                fuzzyMin(output.kekanan, mf_turn_right_out(x))
            ),
            fuzzyMin(output.kebelakang, mf_backward_out(x)) 
        );

        numerator += x * combined;
        denominator += combined;

        }

    if (denominator == 0) return 0.0;
    return numerator / denominator;
}

long d1, d2, d3, d4;

enum DIRECTION {
    MUNDUR = 0,
    KIRI = 1,
    KANAN = 2,
    MAJU = 3
};

DIRECTION direction = MAJU;

void setup() {
    lcd.init();
    lcd.backlight();
    Serial.begin(9600);

    pinMode(depanTrig, OUTPUT); pinMode(depanEcho, INPUT);
    pinMode(kananTrig, OUTPUT); pinMode(kananEcho, INPUT);
    pinMode(kiriTrig, OUTPUT); pinMode(kiriEcho, INPUT);
    pinMode(belakangTrig, OUTPUT); pinMode(belakangEcho, INPUT);
}

void loop() {
    unsigned long currentMillis = millis();

    if(currentMillis - previousMillis_sensor >= interval_sensor) {
    previousMillis_sensor = currentMillis;
    d1 = readUltrasonic(depanTrig, depanEcho);
    d2 = readUltrasonic(kananTrig, kananEcho);
    d3 = readUltrasonic(kiriTrig, kiriEcho);
    d4 = readUltrasonic(belakangTrig, belakangEcho);


    FuzOutPre result = fuzinf(d1, d2, d3, d4);
    float final_value = DefFuzCOG(result);

    if (final_value < -50.0f) {
        direction = MUNDUR;
    } else if (final_value < -15.0f) {
        direction = KIRI;
    } else if (final_value > 15.0f) {
        direction = KANAN;
    } else {
        direction = MAJU;
    }

    Serial.println((int)direction); 

    }

    if(currentMillis - previousMillis_gait >= interval_gait) {
    previousMillis_gait = currentMillis;
        switch (direction) {
            case MAJU:
                lcd.setCursor(0, 0);
                lcd.print("MAJU  ");
                break;
            case KIRI:
                lcd.setCursor(0, 1);
                lcd.print("KIRI  ");
                break;
            case KANAN:
                lcd.setCursor(0, 1);
                lcd.print("KANAN ");
                break;
            case MUNDUR:
                lcd.setCursor(0, 0);
                lcd.print("MUNDUR");
                break;
        }
    }
    delay(1000);
    lcd.clear();
}
