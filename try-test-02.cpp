#include <string.h>

#define trig_depan 6
#define echo_depan 7
#define trig_kanan 8
#define echo_kanan 9
#define trig_kiri 10
#define echo_kiri 11
#define trig_belakang 12
#define echo_belakang 13

#define b println("jarak depan = ")
#define c println("jarak kanan = ")
#define n println("jarak kiri = ")
#define j println("jarak belakang = ")
#define p Serial

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

String arah(long depan,long kanan,long kiri,long belakang){
    String arah ="";
    if (depan > 10 && kanan < 10 && kiri < 10 && belakang < 10 ) {
        arah = "maju";
    } else if(depan > 10 && kanan < 10 && kiri > 10 && belakang < 10 ) {
        arah = "serong kanan";
    } else if(depan > 10 && kanan > 10 && kiri < 10 && belakang < 10 ) {
        arah = "serong kiri";
    } else if(depan > 10 && kanan < 10 && kiri > 10 && belakang > 10 ) {
        arah = "kanan";
    } else if(depan > 10 && kanan > 10 && kiri < 10 && belakang > 10 ) {
        arah = "kiri";
    } else if(depan < 10 && kanan < 10 && kiri < 10 && belakang > 10 ) {
        arah = "belakang";
    } 

    return arah;
}  

void setup() {
  // put your setup code here, to run once:
  pinMode(trig_depan, OUTPUT);
  pinMode(echo_depan, INPUT);
  pinMode(trig_kanan, OUTPUT);
  pinMode(echo_kanan, INPUT);
  pinMode(trig_kiri, OUTPUT);
  pinMode(echo_kiri, INPUT);
  pinMode(trig_belakang, OUTPUT);
  pinMode(echo_belakang, INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  long depan = readUltrasonic(trig_depan,echo_depan);
  p.b;
  p.println(depan);
  long kanan = readUltrasonic(trig_kanan,echo_kanan);
  p.c;
  p.println(kanan);
  long kiri = readUltrasonic(trig_kiri,echo_kiri);
  p.n;
  p.println(kiri);
  long belakang = readUltrasonic(trig_belakang,echo_belakang);
  p.j;
  p.println(belakang);

  String gerak = arah(depan,kanan,kiri,belakang);
  p.println("arah : ");
  p.println(gerak);
  delay(1000);  
}
