SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#include "TB6612FNG.h";           // my motor driver
#include "SparkFunMMA8452Q.h";    // sparkfun accelerometer driver
#include "neopixel.h";            // neopixel driver

//neo Pixel Definitions
#define          PIXEL_PIN D7
#define          PIXEL_COUNT 2
#define          PIXEL_TYPE WS2812B
void             colorAll(uint32_t c, uint8_t wait);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

//pins
#define          MK1_MANUALCONTROLBUTTON A2
#define          MK1_SERVOPIN WKP
#define          MK1_DISTANCESENSOR A0
#define          MK1_TOFSENSOR A1
#define          MK1_SPEAKERPIN A5

//collision detection
#define          MK1_MINCOLLISIONDISTANCE 70
#define          MK1_MINSWEEPCOLLISIONTIMER 7500

//cruise speed
#define          MK1_CRUISESPEED 180

//basic mk1 commands
bool             mk1_command = false;
bool             mk1_obstacleDetected = false;
bool             mk1_cliffDetected = false;
String           mk1_direction;
int              mk1_automated_turn = 0;
int              mk1_last_siren_mode = 1;

//obstacle sweep globals
unsigned long    mk1_last_obstacle_avoided = 0;
unsigned long    mk1_obstacle_currentMillis = 0;
unsigned long    mk1_previous_obstacle_millis = 0;

//manual start button
volatile long    mk1_manualControlButton_lastPress = 0;

//timers
Timer            mk1_obstacleTimer(50, detectObstacle);
Timer            mk1_cliffTimer(50, detectCliff);
Timer            mk1_sirenTimer(250, runSirens);
Timer            mk1_beepTimer(500, runBeep);

//servo
Servo mk1_servo;

//instance of accelerometer
MMA8452Q         accel;
double           mk1_accelThreshold = 0.7;

//Time of flight sensor
int              mk1_tof;

//instance of motor controller
Motors motorController(D4, TX, D2, D3, RX, D5, D6);

void setup(){
  //enable manual control through the button
  pinMode(MK1_MANUALCONTROLBUTTON, INPUT_PULLUP);

  //reset the position of the servo
  resetServo();

  //start the timers
  mk1_obstacleTimer.start();
  mk1_cliffTimer.start();
  mk1_sirenTimer.start();
  mk1_beepTimer.start();

  //initialize neopixels
  strip.begin();
  strip.show();

  //initialize the accelerometer
  accel.begin(SCALE_2G, ODR_100);
  delay(1000);

  //enable serial communication
  Serial.begin(9600);

  //connect and register particle cloud functions
  Particle.function("move", setDirection);
  Particle.connect();

  //startup sound
  tone(MK1_SPEAKERPIN, 500, 200);
  delay(100);
  tone(MK1_SPEAKERPIN, 1000, 200);
  delay(100);
  tone(MK1_SPEAKERPIN, 1500, 200);
}
void loop(){
  bool manualControlCommand = manualControl();
  bool remoteControlCommand = remoteControl();
  bool avoidCliffCommand = avoidCliff();
  bool detectImpactCommand = detectImpact();
  bool avoidObstacleSweepCommand = avoidObstacleSweep();
  bool avoidObstacleCommand = avoidObstacle();
  bool cruiseCommand = cruise();

  if(manualControlCommand) manualControlAction();
  else if(remoteControlCommand) remoteControlAction();
  else if(avoidCliffCommand) avoidCliffAction();
  else if(detectImpactCommand) detectImpactAction();
  else if(avoidObstacleSweepCommand) avoidObstacleSweepAction();
  else if(avoidObstacleCommand) avoidObstacleAction();
  else if(cruiseCommand) cruiseAction();
  else stop();

  if (accel.available()){
       accel.read();
       Serial.println(accel.cy);
   }
}

bool manualControl(){
  if(digitalRead(MK1_MANUALCONTROLBUTTON) == LOW) {
  unsigned long button_press = millis();
  if(button_press -   mk1_manualControlButton_lastPress >= 1500){
        mk1_manualControlButton_lastPress = button_press;
      return true;
    }else{
      return false;
    }
  }else{
    return false;
  }
}

void manualControlAction(){
  if(mk1_command == false){
    setDirection("c");
  }else{
    stop();
    mk1_command = false;
  }
}
bool detectImpact(){
   if(mk1_command == true && accel.available() && millis() > 10000){
      accel.read();
     if(accel.cy < -mk1_accelThreshold || accel.cy > mk1_accelThreshold){
       Serial.print("impact detected");
       return true;
     }else{
       return false;
     }
  }else{
    return false;
  }
}

void detectImpactAction(){
  motorController.reverse(255);
  delay(500);
  motorController.right(255);
  delay(500);
}

bool avoidObstacleSweep(){
  mk1_obstacle_currentMillis = millis();
  if(mk1_command == true && mk1_obstacle_currentMillis - mk1_last_obstacle_avoided >= MK1_MINSWEEPCOLLISIONTIMER && mk1_obstacleDetected == true){
    return true;
  }else{
    return false;
  }
}
void avoidObstacleSweepAction(){
  int forward_distance, right_45_distance, right_90_distance, left_45_distance, left_90_distance;
  forward_distance = findDistance();
  motorController.stop();
  delay(100);
  mk1_servo.attach(MK1_SERVOPIN);

  mk1_servo.write(0);
  delay(500);
  left_90_distance = findDistance();
  delay(600);

  mk1_servo.write(45);
  delay(500);
  left_45_distance = findDistance();
  delay(600);

  mk1_servo.write(135);
  delay(500);
  right_45_distance = findDistance();
  delay(600);

  mk1_servo.write(180);
  delay(600);
  right_90_distance = findDistance();
  delay(500);

  mk1_servo.write(90);
  delay(500);
  mk1_servo.detach();
  if(forward_distance >= right_90_distance && forward_distance >= left_90_distance && forward_distance >= left_45_distance && forward_distance >= right_45_distance){
    motorController.forward(MK1_CRUISESPEED);
  }
  if(left_90_distance > right_90_distance && left_90_distance > left_45_distance && left_90_distance > right_45_distance){
    mk1_automated_turn = 16; //set the next automated turn to be left
    motorController.left(255);
    delay(300);
  }else if(left_45_distance > right_90_distance && left_45_distance > left_90_distance && left_45_distance > right_45_distance) {
    mk1_automated_turn = 16; //set the next automated turn to be left
    motorController.left(255);
    delay(150);
  }else if(right_45_distance > right_90_distance && right_45_distance > left_90_distance && right_45_distance > left_45_distance) {
    mk1_automated_turn = 0; //set the next automated turn to be right
    motorController.right(255);
    delay(150);
  }else{
    mk1_automated_turn = 0; //set the next automated turn to be right
    motorController.right(255);
    delay(300);
  }
  resetServo();
  motorController.forward(MK1_CRUISESPEED);
  delay(300);
  mk1_command = true;
  mk1_last_obstacle_avoided = millis();
}

bool avoidObstacle(){
  if(mk1_command == true) return mk1_obstacleDetected;
  else return false;
}
void avoidObstacleAction(){
  if(mk1_automated_turn <=15){
    motorController.right(255);
  }else{
    motorController.left(255);
  }
  delay(200);
  if(mk1_automated_turn >= 25){
    mk1_automated_turn = 0;
  }else{
    mk1_automated_turn++;
  }
  mk1_last_obstacle_avoided = millis();
}
bool remoteControl(){
  if(mk1_direction != "c" && mk1_direction != "" && mk1_command == true){
    return true;
  }else{
    return false;
  }
}
void remoteControlAction(){
    if(mk1_direction == "z"){
      stop();
    }else if(mk1_direction == "f"){
      colorAll(strip.Color(128, 128, 128), 1); //turn on the headlight on white when driving
      motorController.forward(255);
    }else if(mk1_direction == "r"){
      motorController.right(255);
    }else if(mk1_direction == "l"){
      motorController.left(255);
    }else if(mk1_direction == "b"){
      colorAll(strip.Color(241, 231, 28), 1);
      motorController.reverse(255);
      delay(500);
      stop();
    }
}
bool avoidCliff(){
  if(mk1_command == true) return mk1_cliffDetected;
  else return false;
}

void avoidCliffAction(){
  mk1_automated_turn = 0;
  motorController.reverse(255);
  delay(500);
  motorController.right(255);
  delay(500);
}

bool detectCliff(){
  int mk1_tof = digitalRead(MK1_TOFSENSOR);
  //Serial.println(mk1_tof);
  if(mk1_tof == 0){
    mk1_cliffDetected = true;
  }else{
    mk1_cliffDetected = false;
  }
}

bool cruise(){
  if(mk1_command == true && mk1_direction == "c"){
    return true;
  }else{
    return false;
  }
}

bool cruiseAction(){
  motorController.forward(MK1_CRUISESPEED);
  colorAll(strip.Color(128, 128, 128), 1); //turn on the headlight on white when driving
}

int setDirection(String direction){
  mk1_direction = direction;
  mk1_command = true;
  return false;
}

int findDistance(){
  int sensor, inches, x;
  sensor = analogRead(MK1_DISTANCESENSOR);  // read the analog output of the EZ1 from analog input 0
  return sensor / 2;        // convert the sensor reading to inches
}

void detectObstacle(){
  int inches = findDistance();
  Serial.println(inches);
  if(inches <= MK1_MINCOLLISIONDISTANCE){
    mk1_obstacleDetected = true;
  }else{
    mk1_obstacleDetected = false;
  }
}

//stop the vehicle
void stop(){
  motorController.stop();       //stop the motors
  mk1_command = false;
  colorAll(strip.Color(5, 5, 5), 1);
}
//set the servo back to middle position
void resetServo(){
  mk1_servo.attach(MK1_SERVOPIN);  //Initialize the servo attached to pin D0
  mk1_servo.write(90);             //set servo to furthest position
  delay(500);                      //delay to give the servo time to move to its position
  mk1_servo.detach();              //detach the servo to prevent it from jittering
}

void alternateSirenColor(){
  if(mk1_last_siren_mode == 0){
    mk1_last_siren_mode = 1;
  }else{
    mk1_last_siren_mode = 0;
  }
}

//flash the healights alternating between red and white
void runSirens(){
  if(mk1_obstacleDetected && mk1_command == true && mk1_direction == "c"){
    if(mk1_last_siren_mode == 0){
      strip.setPixelColor(0, 85, 85, 85);
      strip.setPixelColor(1, 255, 0, 0);
      strip.show();
      mk1_last_siren_mode = 1;
    }else{
      strip.setPixelColor(0, 255, 0, 0);
      strip.setPixelColor(1, 85, 85, 85);
      strip.show();
      mk1_last_siren_mode = 0;
    }
  }
}

void runBeep(){
  if(mk1_obstacleDetected && mk1_command == true && mk1_direction == "c"){
    tone(MK1_SPEAKERPIN, 1200, 200);
  }
}

//neopixel basic color handling
void colorAll(uint32_t c, uint8_t wait) {
  uint16_t i;

  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}
