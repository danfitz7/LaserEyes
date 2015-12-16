/* Arduino code for MAS.S65 Science Fiction Fabrication Class, Fall 2015
 * Daniel Fitzgerald
 * 
 * The project has two parts: 
 *  1) A head-mounted 2-DOF laser pointer gimbal linked to a head-worn gaze tracker.
 *  The laser points where the user is looking
 *  
 *  2) A Shoulder-mounted laser-tracking nerf turret. The turret shoots where the laser points.
 *  This turret can also be mounted on a standard camera tripod and used as an automatic laser-guided sentry turret.
 *
 *
 * TODO
 * - Use Servo.microseconds() instead of aiming with angles (more precise, higher resolution, probably faster)
 * Needs custom calibration per servo type.
 * - Add timer interrupts. When a fire command is encountered, turn on the gun and start an interrupt to trigger TURRET_FIRE_TIME later
 * When the interrupt trggers, turn the gun off and dissable the interrupt.
 * This way, the turret can keep tracking as it's firing.
 * - Do the same for fade in/out
 */

#include <Servo.h>

// #define VERBOSE // uncomment for verbose debug output

// For manual calibration with a potentiometer
//#define potPin A0

// ############################################## SHOULDER TURRET ##############################################

// Shoulder turret pins
#define TURRET_GUN_PIN 2  // RED
#define TURRET_GLOW_PIN 3 //Blue
#define TURRET_LASER_PIN 4//Green
#define TURRET_YAW_PIN 6  //Orange
#define TURRET_YAW_INVERTED
#define TURRET_PITCH_PIN 5//Yellow
#define TURRET_PITCH_INVERTED
Servo turretYawServo;
Servo turretPitchServo;

// Shoulder turret constants
#define TURRET_YAW_MIN 0
#define TURRET_YAW_CENTER 98
#define TURRET_YAW_MAX 140
#define TURRET_PITCH_MIN 40
#define TURRET_PITCH_CENTER 96
#define TURRET_PITCH_MAX 105 

volatile unsigned char turret_glow_val = 0;
void setupTurret(){
  Serial.println("SETUP TURRET");
  
  // First TURN OFF THE GUN!
  pinMode(TURRET_GUN_PIN, OUTPUT);
  digitalWrite(TURRET_GUN_PIN, LOW);

  turretYawServo.attach(TURRET_YAW_PIN);
  turretYawServo.write(90);
  turretPitchServo.attach(TURRET_PITCH_PIN);
  turretPitchServo.write(90);

  pinMode(TURRET_LASER_PIN, OUTPUT);
  digitalWrite(TURRET_LASER_PIN, LOW);

  pinMode(TURRET_GLOW_PIN, OUTPUT);
  analogWrite(TURRET_GLOW_PIN, turret_glow_val);
}

inline void turret_laser_off(){
  #ifdef VERBOSE
    Serial.println("Turret laser ON");
  #endif
  
  digitalWrite(TURRET_LASER_PIN, LOW);
}

inline void turret_laser_on(){
  #ifdef VERBOSE
    Serial.println("Turret laser OFF");
  #endif
  
  digitalWrite(TURRET_LASER_PIN, HIGH);
}

#define TURRET_FIRE_TIME 1000
void turret_fire(){
  #ifdef VERBOSE
    Serial.println("TURRET FIRE!");
  #endif
  
  digitalWrite(TURRET_LASER_PIN, HIGH);
  digitalWrite(TURRET_GUN_PIN, HIGH);
  delay(TURRET_FIRE_TIME);
  digitalWrite(TURRET_GUN_PIN, LOW);
  digitalWrite(TURRET_LASER_PIN, LOW);
}

#define TURRET_GLOW_FADE_SPEED 10
void turret_fade_in(){
  #ifdef VERBOSE
    Serial.println("Turret fade in");
  #endif
    
  for (int i = turret_glow_val;i <= 255;i++){
    analogWrite(TURRET_GLOW_PIN, i);
    delay(TURRET_GLOW_FADE_SPEED);
  }
  turret_glow_val = 255;
}

void turret_fade_out(){
  #ifdef VERBOSE
    Serial.println("Turret fade out");
  #endif
  
  for (int i = turret_glow_val;i >= 0;i--){
    analogWrite(TURRET_GLOW_PIN, i);
    delay(TURRET_GLOW_FADE_SPEED);
  }
  turret_glow_val = 0;
}

// Takes yaw/pitch coordinates in turret reference frame, constrains them within turrent safe range, and aims at that position
inline void turret_go_to(int yaw, int pitch){
  #ifdef VERBOSE
    Serial.print("Turret go to (");
    Serial.print(yaw);
    Serial.print(", ");
    Serial.print(pitch);
    Serial.println(")");
  #endif
  
  int newYaw = constrain(yaw, TURRET_YAW_MIN, TURRET_YAW_MAX);
  int newPitch = constrain(pitch, TURRET_PITCH_MIN, TURRET_PITCH_MAX);

  #ifdef VERBOSE
    if (newYaw != yaw || newPitch != pitch){
      Serial.print("\tTurret constrained at (");
      Serial.print(yaw);
      Serial.print(", ");
      Serial.print(pitch);
      Serial.println(")");
    }
  #endif
  
  turretYawServo.write(newYaw);
  turretPitchServo.write(newPitch);
}

// Aim function relative to "true" (camera) turret coorinates where (0,0) is center
void turret_aim(int yaw, int pitch){
  #ifdef TURRET_YAW_INVERTED
    yaw = -yaw;
  #endif
  #ifdef TURRET_PITCH_INVERTED
    pitch = -pitch;
  #endif
  
  yaw += TURRET_YAW_CENTER;
  pitch += TURRET_PITCH_CENTER;

  turret_go_to(yaw, pitch);
}

void turret_deactivate(){
  #ifdef VERBOSE
    Serial.println("TURRET DEACTIVATED");
  #endif
  
  turret_aim(0,50);
  turret_fade_out();
  turret_aim(0,45);
  turretYawServo.detach();
  turretPitchServo.detach();
}

void turret_activate(){
  #ifdef VERBOSE
    Serial.println("TURRET ACTIVATED");
  #endif
  
  turretYawServo.attach(TURRET_YAW_PIN);
  turretPitchServo.attach(TURRET_PITCH_PIN);
  turret_aim(0,0);
  turret_fade_in();
}

void testTurret(){
  Serial.println("TEST TURRET");
  
  turret_fade_in();

  turret_laser_off();
  delay(1000);
  turret_laser_on();
  
  #define turret_wait_time 2000
  turret_aim(0,0); 
  delay(turret_wait_time);
  
  turret_aim(-45,0);
  delay(turret_wait_time);
  turret_aim(45, 0);
  delay(turret_wait_time);

  turret_aim(0,0); 
  delay(turret_wait_time);
  
  turret_aim(0, -45);
  delay(turret_wait_time);
  turret_aim(0, 45);
  delay(turret_wait_time);
  turret_aim(0, 0);
  delay(turret_wait_time);

  turret_fire();

  turret_fade_out();
}

// ############################################## HEAD LASER GIMBAL ##############################################

// Head laser pins
#define GIMBAL_LASER_PIN 7
#define GIMBAL_YAW_PIN 8
//#define GIMBAL_YAW_INVERTED
#define GIMBAL_PITCH_PIN 12
//#define GIMBAL_PITCH_INVERTED
Servo gimbalYawServo;
Servo gimbalPitchServo;

// Head laser gimbal constants
#define GIMBAL_YAW_CENTER 91
const unsigned short GIMBAL_YAW_MIN = GIMBAL_YAW_CENTER - 90;//#define GIMBAL_YAW_MIN 45
const unsigned short GIMBAL_YAW_MAX = GIMBAL_YAW_CENTER + 90;//#define GIMBAL_YAW_MAX 135
#define GIMBAL_PITCH_CENTER 50 //102
const unsigned short GIMBAL_PITCH_MIN = GIMBAL_PITCH_CENTER - 50;//#define GIMBAL_PITCH_MIN 45
const unsigned short GIMBAL_PITCH_MAX = GIMBAL_PITCH_CENTER + 50;//#define GIMBAL_PITCH_MAX 135 

void setupGimbal(){
  Serial.println("SETUP LASER GIMBAL");
  
  // First TURN OFF THE LASER!
  pinMode(GIMBAL_LASER_PIN, OUTPUT);
  digitalWrite(GIMBAL_LASER_PIN, LOW);

  gimbalYawServo.attach(GIMBAL_YAW_PIN);
  gimbalYawServo.write(GIMBAL_YAW_CENTER);
  gimbalPitchServo.attach(GIMBAL_PITCH_PIN);
  gimbalPitchServo.write(GIMBAL_PITCH_CENTER);
}

void gimbal_laser_on(){
  #ifdef VERBOSE
    Serial.println("Gimbal laser ON!");
  #endif
  
  digitalWrite(GIMBAL_LASER_PIN, HIGH);
}

void gimbal_laser_off(){
  #ifdef VERBOSE
    Serial.println("Gimbal laser OFF.");
  #endif
  
  digitalWrite(GIMBAL_LASER_PIN, LOW);
}

// Takes yaw/pitch coordinates in gimbal reference frame, constrains them within turrent safe range, and aims at that position
inline void gimbal_go_to(int yaw, int pitch){
  #ifdef VERBOSE
    Serial.print("Gimbal go to (");
    Serial.print(yaw);
    Serial.print(", ");
    Serial.print(pitch);
    Serial.println(")");
  #endif

  int newYaw = constrain(yaw, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
  int newPitch = constrain(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

  #ifdef VERBOSE
    if (newYaw != yaw || newPitch != pitch){
      Serial.print("\tGimbal constrained at (");
      Serial.print(yaw);
      Serial.print(", ");
      Serial.print(pitch);
      Serial.println(")");
    }
  #endif
  
  gimbalYawServo.write(newYaw);
  gimbalPitchServo.write(newPitch);
}

// Laser Gimbal aim function relative to "true" (head) coorinates where (0,0) is center
void gimbal_aim(int yaw, int pitch){
  #ifdef GIMBAL_YAW_INVERTED
    yaw = -yaw;
  #endif
  #ifdef GIMBAL_PITCH_INVERTED
    pitch = -pitch;
  #endif
  
  yaw += GIMBAL_YAW_CENTER;
  pitch += GIMBAL_PITCH_CENTER;

  gimbal_go_to(yaw, pitch);
}

void testGimbal(){
  Serial.println("TEST GIMBAL");

  gimbal_laser_on();
  
  #define gimbal_wait_time 2000
  gimbal_aim(0,0); 
  delay(turret_wait_time);
  
  gimbal_aim(-45,0);
  delay(turret_wait_time);
  gimbal_aim(45, 0);
  delay(turret_wait_time);

  gimbal_aim(0,0); 
  delay(turret_wait_time);
  
  gimbal_aim(0, -45);
  delay(turret_wait_time);
  gimbal_aim(0, 45);
  delay(turret_wait_time);
  gimbal_aim(0, 0);
  delay(turret_wait_time);

  gimbal_laser_off();
}

// ############################################## COMMAND MESSAGE PARSING ##############################################

// TURRET COMMANDS
#define TURRET_COMMAND_FRAME_START 'T'
#define TURRET_FIRE_MESSAGE_INDICTOR 'F'
#define TURRET_AIM_MESSAGE_INDICATOR 'A'
#define TURRET_ACTIVATE_MESSAGE_INDICATOR 'X'
#define TURRET_DEACTIVATE_MESSAGE_INDICATOR 'O'

// LASER GIMBAL COMMANDS
#define GIMBAL_COMMAND_FRAME_START 'G'
#define GIMBAL_LASERON_MESSAGE_INDICTOR 'L'
#define GIMBAL_LASEROFF_MESSAGE_INDICTOR 'l'
#define GIMBAL_AIM_MESSAGE_INDICATOR 'A'

inline boolean isValidCommandFrameStart(char a){
  return (a == TURRET_COMMAND_FRAME_START || a == GIMBAL_COMMAND_FRAME_START);
}
int numBytesAvailable = 0;

// Packs the next 4 bytes in the Serial buffer in to 2 shorts (2-bytes each)
// WARNING: This should only be called if there are actually at-least 4 bytes in the buffer (Serial.available() >= 4 has just been verified)
#define COORDINATE_MESSAGE_DATA_LENGTH 4
void parseCoordinates(short* pitch, short* yaw){
  
  // Parse four bytes (2 ints, 2 bytes each)
  byte firstByte = Serial.read();
  byte secondByte= Serial.read();
  byte thirdByte = Serial.read();
  byte fourthByte= Serial.read();

  // This accomplishes bit shifting to reconstruct the 2-byte shirts
  short firstNum = firstByte * 256 + secondByte; // Yaw
  short secondNum = thirdByte * 256 + fourthByte;// Pitch

  *pitch = firstNum;
  *yaw = secondNum;
}

void getToNextCommandStart(){
  // Clear bytes until we get to a valid command start
  char nextByte = Serial.peek();
  while (!isValidCommandFrameStart(nextByte) && numBytesAvailable >= 2){
    Serial.read();
    numBytesAvailable = Serial.available();
    nextByte = Serial.peek();
  }
}

inline void parseTurretCommand(){
  Serial.read(); // clear the turret command frame start byte
  numBytesAvailable = Serial.available();
  
  #ifdef VERBOSE
    Serial.println("Parsing turret command...");
  #endif

  // Fire command
  char nextChar = Serial.peek();
  if (nextChar == TURRET_FIRE_MESSAGE_INDICTOR){
    #ifdef VERBOSE
      Serial.println("Turret FIRE message.");
    #endif
    
    Serial.read(); // clear the turret fire message indicator
    turret_fire();
   
  // Aim command  
  }else if (nextChar == TURRET_AIM_MESSAGE_INDICATOR){
    #ifdef VERBOSE
      Serial.println("Turret AIM message...");
    #endif
    
    Serial.read(); // clear the turret aim message indicator
    while (Serial.available() < (COORDINATE_MESSAGE_DATA_LENGTH)){
      delay(1); // wait for new all the data to come in
    }
    
    short turretYaw, turretPitch;
    parseCoordinates(&turretYaw, &turretPitch);
    turret_aim(turretYaw, turretPitch); 
     
  }else if (nextChar == TURRET_DEACTIVATE_MESSAGE_INDICATOR){
    Serial.read();
    turret_deactivate();
    
  }else if (nextChar == TURRET_ACTIVATE_MESSAGE_INDICATOR){
    Serial.read();
    turret_activate();
  }
}

inline void parseGimbalCommand(){
  Serial.read(); // clear the laser command frame start byte
  numBytesAvailable = Serial.available();
  
  #ifdef VERBOSE
    Serial.println("Parsing laser gimbal command...");
  #endif
  
  // laser ON command
  if (Serial.peek() == GIMBAL_LASERON_MESSAGE_INDICTOR){
    #ifdef VERBOSE
      Serial.println("Gimbal Laser ON message.");
    #endif
    
    Serial.read(); // clear the laser on message indicator
    gimbal_laser_on();

  // laser OFF
  }else if (Serial.peek() == GIMBAL_LASEROFF_MESSAGE_INDICTOR){
    #ifdef VERBOSE
      Serial.println("Gimbal Laser OFF message.");
    #endif
    
    Serial.read(); // clear the laser off message indicator
    gimbal_laser_off();
    
  // Aim command  
  }else if (Serial.peek() == GIMBAL_AIM_MESSAGE_INDICATOR){
    #ifdef VERBOSE
      Serial.println("Gimbal AIM message...");
    #endif
    
    Serial.read(); // clear the turret aim message indicator
    while (Serial.available() < (COORDINATE_MESSAGE_DATA_LENGTH)){
      delay(1); // wait for new all the data to come in
    }
    
    short laserYaw, laserPitch;
    parseCoordinates(&laserYaw, &laserPitch);
    gimbal_aim(laserYaw, laserPitch);  
    
  } // Possible other types of laser messages
}

inline void processSerialMessages(){
  // Check if there's any possible commands waiting for us
  numBytesAvailable = Serial.available();
  if (numBytesAvailable >= 2){

    getToNextCommandStart();

    // if we're receiveing a turret command
    if (Serial.peek() == TURRET_COMMAND_FRAME_START){
      parseTurretCommand();

    // if we received a laser command  
    }else if (Serial.peek() == GIMBAL_COMMAND_FRAME_START){
      parseGimbalCommand();
    }
    
  } // end check available bytes
}

// ############################################## MAIN PROGRAM ##############################################

void setup() {
  Serial.begin(115200);
  setupTurret();
  setupGimbal();

  Serial.println("LASER GIMBAL ACTIVATED");
  gimbal_aim(0,0);

  #ifdef potPin
    pinMode(potPin, INPUT);
    gimbal_laser_on();
    turret_laser_on();
  #endif

  //testGimbal();
  //delay(2000);
  //testTurret();

  turret_deactivate();
}

void loop() {
  #ifdef potPin
    int val = map(analogRead(potPin), 0, 1023, 0, 360);
    Serial.println(val);
    turret_go_to(TURRET_YAW_CENTER, val);
  #else
    processSerialMessages();
  #endif
}
