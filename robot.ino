SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#include                  "TB6612FNG.h";           // my motor driver
#include                  "SparkFunMMA8452Q.h";    // sparkfun accelerometer driver
#include                  "neopixel.h";            // neopixel driver

//enable debugging

#define                   DEBUG false

//neo Pixel Definitions
#define                   PIXEL_PIN D7
#define                   PIXEL_COUNT 2
#define                   PIXEL_TYPE WS2812B
void                      colorAll(uint32_t c, uint8_t wait);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

//pins
#define                   MK1_MODEBUTTON A2
#define                   MK1_MANUALCONTROLBUTTON A3
#define                   MK1_SERVOPIN WKP
#define                   MK1_DISTANCESENSOR A0
#define                   MK1_TOFSENSOR A1
#define                   MK1_SPEAKERPIN A5

//collision detection
#define                   MK1_MINCOLLISIONDISTANCE 70
#define                   MK1_MINCOLLISIONDISTANCE_IR 228
#define                   MK1_MINSWEEPCOLLISIONTIMER 1500

//cruise speed
#define                   MK1_CRUISESPEED 200

//basic mk1 commands
bool                      mk1_command = false;
bool                      mk1_obstacleDetected = false;
bool                      mk1_cliffDetected = false;
String                    mk1_direction;
int                       mk1_automated_turn = 0;
int                       mk1_last_siren_mode = 1;

//set the default control mode to 0 upon startup
int                       mk1_control_mode = 0;

unsigned int              mk1_ext_light_sensor_front = 0;
unsigned int              mk1_ext_light_sensor_right = 0;
unsigned int              mk1_ext_light_sensor_left = 0;
unsigned long             mk1_ext_light_last_millis = 0;
int                       mk1_last_sensor_check_millis = 0;
int                       mk1_ext_left_ping = 0;
int                       mk1_ext_right_ping = 0;

//obstacle sweep globals
unsigned long             mk1_last_obstacle_avoided = 0;
unsigned long             mk1_obstacle_currentMillis = 0;
unsigned long             mk1_previous_obstacle_millis = 0;

//manual start button
volatile long             mk1_manualControlButton_lastPress = 0;

//manual mode button
volatile long             mk1_manualModeButton_lastPress = 0;

//timers
Timer                     mk1_obstacleTimer(50, detectObstacle);
//Timer                   mk1_cliffTimer(50, detectCliff);
Timer                     mk1_sirenTimer(250, runSirens);
Timer                     mk1_beepTimer(500, runBeep);

//servo
Servo mk1_servo;

//instance of accelerometer
MMA8452Q                  accel;
double                    mk1_accelThreshold = 1.85;

//Time of flight sensor
int                       mk1_tof;

//instance of motor controller
Motors motorController(D4, A4, D2, D3, RX, D5, D6);

void setup(){
  //initialize neopixels
  strip.begin();
  strip.show();
  headlights("green");

  Wire.begin();
  Serial.begin(9600);

  //manual start/stop button
  pinMode(MK1_MANUALCONTROLBUTTON, INPUT_PULLUP);

  //mode change button
  pinMode(MK1_MODEBUTTON, INPUT_PULLUP);

  //reset the position of the servo
  resetServo();

  //start the timers
  mk1_obstacleTimer.start();
  //mk1_cliffTimer.start();
  mk1_sirenTimer.start();
  mk1_beepTimer.start();

  //initialize the accelerometer
  accel.begin(SCALE_2G, ODR_100);
  delay(1000);

  //connect and register cloud functions
  Particle.function("move", setDirection);
  Particle.function("mode", setMode);
  Particle.connect();

  //startup sound
  tone(MK1_SPEAKERPIN, 500, 200);
  headlights("yellow");
  delay(100);
  tone(MK1_SPEAKERPIN, 1000, 200);
  headlights("red");
  delay(100);
  tone(MK1_SPEAKERPIN, 1500, 200);
}
void loop(){
  getExtSensorValues();

  bool manualControlCommand = manualControl();
  bool manualModeCommand = manualModeChange();
  bool remoteControlCommand = remoteControl();
  //bool avoidCliffCommand = avoidCliff();
  bool detectImpactCommand = detectImpact();
  bool followWallCommand = followWall();
  bool avoidObstacleSweepCommand = avoidObstacleSweep();
  bool avoidObstacleCommand = avoidObstacle();
  bool lightControlCommand = followLight();
  bool cruiseCommand = cruise();

  if(mk1_control_mode == 0){
    if(manualControlCommand) manualControlAction();
    else if(manualModeCommand) manualModeChangeAction();
    else if(remoteControlCommand) remoteControlAction();
    //else if(avoidCliffCommand) avoidCliffAction();
    else if(avoidObstacleSweepCommand) avoidObstacleSweepAction();
    else if(avoidObstacleCommand) avoidObstacleAction();
    else if(detectImpactCommand) detectImpactAction();
    else if(cruiseCommand) cruiseAction();
    else stop();
  }else if(mk1_control_mode == 1){
    if(manualControlCommand) manualControlAction();
    else if(manualModeCommand) manualModeChangeAction();
    else if(remoteControlCommand) remoteControlAction();
    //else if(avoidObstacleCommand) avoidObstacleAction();
    else if(detectImpactCommand) detectImpactAction();
    else if(lightControlCommand) followLightAction();
  }else if(mk1_control_mode == 2){
    if(manualControlCommand) manualControlAction();
    else if(manualModeCommand) manualModeChangeAction();
    else if(remoteControlCommand) remoteControlAction();
    else if(detectImpactCommand) detectImpactAction();
    else if(followWallCommand) followWallAction();
  }

  if(mk1_control_mode == 1){
    headlights("off");
  }else if(mk1_control_mode != 1 && mk1_command == false){
    headlights("dim_white");
  }
}
void getExtSensorValues(){
  unsigned long last_sensor_check = millis();
  if(last_sensor_check - mk1_last_sensor_check_millis >= 100){
    mk1_last_sensor_check_millis = last_sensor_check;
    Wire.requestFrom(6,5);
    mk1_ext_light_sensor_front = Wire.read();
    mk1_ext_light_sensor_right = Wire.read();
    mk1_ext_light_sensor_left = Wire.read();
    mk1_ext_right_ping = Wire.read();
    mk1_ext_left_ping = Wire.read();
    if(DEBUG == true){
      Serial.println("-------");
      Serial.println(mk1_ext_light_sensor_front);
      Serial.println(mk1_ext_light_sensor_right);
      Serial.println(mk1_ext_light_sensor_left);
      Serial.println(mk1_ext_left_ping);
    }
  }
}

bool followWall(){
  if(mk1_control_mode == 2 && mk1_command == true){
    return true;
  }else{
    return false;
  }
}

void followWallAction(){
  if(mk1_obstacleDetected == true){
    motorController.right(255);
    delay(300);
  }else if(mk1_ext_left_ping > 12){
    motorController.left(255);
    delay(40);
    motorController.forward(MK1_CRUISESPEED);
    delay(160);
  }else if(mk1_ext_left_ping >= 2 && mk1_ext_left_ping < 12){
    motorController.right(255);
    delay(550);
  }else{
    motorController.forward(MK1_CRUISESPEED);
    delay(200);
  }
}

bool followLight(){
  return true;
}

void followLightAction(){
  if(mk1_command == true){
    if(mk1_ext_light_sensor_front > mk1_ext_light_sensor_right && mk1_ext_light_sensor_front > mk1_ext_light_sensor_left){
      motorController.forward(MK1_CRUISESPEED);
      delay(50);
    }else if(mk1_ext_light_sensor_left > mk1_ext_light_sensor_front && mk1_ext_light_sensor_left > mk1_ext_light_sensor_right){
      motorController.right(255);
      delay(50);
    }else if(mk1_ext_light_sensor_right > mk1_ext_light_sensor_front && mk1_ext_light_sensor_right > mk1_ext_light_sensor_left){
      motorController.left(255);
      delay(50);
    }else{
      motorController.stop();
    }
  }
}

bool manualControl(){
  if(digitalRead(MK1_MANUALCONTROLBUTTON) == LOW) {
    unsigned long button_press = millis();
    if(button_press - mk1_manualControlButton_lastPress >= 250){
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
    delay(750);
    setDirection("c");
    mk1_last_obstacle_avoided = millis();
  }else{
    stop();
    mk1_command = false;
  }
}

bool manualModeChange(){
  if(digitalRead(MK1_MODEBUTTON) == LOW) {
    unsigned long button_press = millis();
    if(button_press - mk1_manualModeButton_lastPress >= 250){
      mk1_manualModeButton_lastPress = button_press;
      return true;
    }else{
      return false;
    }
  }else{
    return false;
  }
}

bool manualModeChangeAction(){
  if(mk1_control_mode == 0 || mk1_control_mode == 1){
    mk1_control_mode++;
  }else{
    mk1_control_mode = 0;
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
  if(mk1_control_mode != 1){
    if(mk1_command == true && mk1_obstacle_currentMillis - mk1_last_obstacle_avoided >= MK1_MINSWEEPCOLLISIONTIMER && mk1_obstacleDetected == true){
      return true;
    }else{
      return false;
    }
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
  /*if(mk1_automated_turn <=15){*/
  if((mk1_ext_left_ping >= 2 && mk1_ext_left_ping < 12) || mk1_automated_turn <=30){
    motorController.left(255);
    delay(250);
  /*}else if(mk1_obstacle_currentMillis - mk1_last_obstacle_avoided >= 150){*/
  }else{
    motorController.right(255);
    delay(250);
  }
  if(mk1_automated_turn >= 45){
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
      headlights("white");
      motorController.forward(255);
    }else if(mk1_direction == "r"){
      headlights("white");
      motorController.right(255);
    }else if(mk1_direction == "l"){
      headlights("white");
      motorController.left(255);
    }else if(mk1_direction == "b"){
      headlights("yellow");
      motorController.reverse(255);
      delay(500);
      headlights("dim_white");
      stop();
    }
}

void headlights(String color){
  if(mk1_control_mode != 1){
    if(color == "white"){
      if(mk1_control_mode == 0){
        colorAll(strip.Color(128, 128, 128), 1); //white headlight
      }else{
        colorAll(strip.Color(80, 80, 255), 1); //sligtly blue headlights
      }
    }else if(color == "dim_white"){
      if(mk1_control_mode == 0){
        colorAll(strip.Color(6, 6, 6), 1);
      }else{
        colorAll(strip.Color(0, 0, 12), 1);
      }
    }else if(color == "yellow"){
      colorAll(strip.Color(241, 231, 28), 1);
    }else if(color == "red"){
      colorAll(strip.Color(10, 0, 0), 1);
    }else if(color == "green"){
      colorAll(strip.Color(0, 10
        , 0), 1);
    }else if(color == "off"){
      colorAll(strip.Color(0, 0, 0), 1);
    }
  }else{
    colorAll(strip.Color(0, 0, 0), 1);
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

  if(DEBUG == TRUE){
    Serial.print("TOF: ");
    Serial.println(mk1_tof);
  }
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
  headlights("white"); //turn on the headlight on white when driving
  mk1_last_obstacle_avoided = millis();
}

int setDirection(String direction){
  mk1_direction = direction;
  mk1_command = true;
  return false;
}

int setMode(String mode){
  if(mode == "0"){
    mk1_control_mode = 0;
  }else if(mode == "1"){
    mk1_control_mode = 1;
    headlights("off");
  }else if(mode == "2"){
    mk1_control_mode = 2;
  }
  return false;
}

int findDistance(){
  int sensor, inches, x;
  sensor = analogRead(MK1_DISTANCESENSOR);  // read the analog output of the EZ1 from analog input 0
  return sensor / 2;        // convert the sensor reading to inches
}

void detectObstacle(){
  int inches = findDistance();
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
  if(mk1_control_mode != 1){
    headlights("dim_white");
  }else{
    headlights("off");
  }
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
  if(mk1_obstacleDetected && mk1_command == true && mk1_direction == "c" && mk1_control_mode != 1){
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
