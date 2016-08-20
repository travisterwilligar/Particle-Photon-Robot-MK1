#include <Wire.h>
#include <NewPing.h>

#define TRIGGER_PIN  8
#define ECHO_PIN     11
#define MAX_DISTANCE 500

  NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
  int uS;
  int ping;

void setup() {
  Serial.begin(9600);
  Wire.begin(6);
  Wire.onRequest(requestEvent);
}

void loop() {  
  uS = sonar.ping_median(5);
  ping = uS / US_ROUNDTRIP_IN;

  requestEvent();
  delay(100);

}

void requestEvent(){
  int lightSensorFront = analogRead(A0);
  int lightSensorRight = analogRead(A2);
  int lightSensorLeft = analogRead(A3);
  byte data[] = { lightSensorFront/4, lightSensorRight/4, lightSensorLeft/4, 0, ping };
  Wire.write(data, 5);

  Serial.println("-------");
  Serial.println(lightSensorFront);
  Serial.println(lightSensorRight);
  Serial.println(lightSensorLeft);
  
}

