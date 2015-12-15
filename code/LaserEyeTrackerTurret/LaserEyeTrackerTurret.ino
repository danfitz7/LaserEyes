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

// For manual calibration with a potentiometer
//#define potPin A0

// ############################################## SHOULDER TURRET ##############################################

// Shoulder turret pins
#define TURRET_GUN_PIN 2  // RED
#define TURRET_GLOW_PIN 3 //Blue
#define TURRET_LASER_PIN 4//Green
#define TURRET_YAW_PIN 6  //Orange
//#define TURRET_YAW_INVERTED
#define TURRET_PITCH_PIN 5//Yellow
//#define TURRET_PITCH_INVERTED
Servo turretYawServo;
Servo turretPitchServo;

// Shoulder turret constants
#define TURRET_YAW_MIN 0
#define TURRET_YAW_CENTER 98
#define TURRET_YAW_MAX 140
#define TURRET_PITCH_MIN 40
#define TURRET_PITCH_CENTER 96
#define TURRET_PITCH_MAX 110 

void setupTurret(){
  // First TURN OFF THE GUN!
  pinMode(TURRET_GUN_PIN, OUTPUT);
  digitalWrite(TURRET_GUN_PIN, LOW);

  Serial.println("SETUP TURRET");

  turretYawServo.attach(TURRET_YAW_PIN);
  turretYawServo.write(90);
  turretPitchServo.attach(TURRET_PITCH_PIN);
  turretPitchServo.write(90);

  pinMode(TURRET_LASER_PIN, OUTPUT);
  digitalWrite(TURRET_LASER_PIN, LOW);

  pinMode(TURRET_GLOW_PIN, OUTPUT);
  analogWrite(TURRET_GLOW_PIN, 0);
}

inline void turret_laser_off(){
  Serial.println("Turret laser ON");
  
  digitalWrite(TURRET_LASER_PIN, LOW);
}

inline void turret_laser_on(){
  Serial.println("Turret laser OFF");
  
  digitalWrite(TURRET_LASER_PIN, HIGH);
}

#define TURRET_FIRE_TIME 1000
void turret_fire(){
  Serial.println("TURRET FIRE!");
  
  digitalWrite(TURRET_LASER_PIN, HIGH);
  digitalWrite(TURRET_GUN_PIN, HIGH);
  delay(TURRET_FIRE_TIME);
  digitalWrite(TURRET_GUN_PIN, LOW);
  digitalWrite(TURRET_LASER_PIN, LOW);
}

#define TURRET_GLOW_FADE_SPEED 10
void turret_fade_in(){
  Serial.println("Turret fade in");
  for (int i=0;i<255;i++){
    analogWrite(TURRET_GLOW_PIN, i);
    delay(TURRET_GLOW_FADE_SPEED);
  }
}

void turret_fade_out(){
  Serial.println("Turret fade out");
  for (int i=255;i>=0;i--){
    analogWrite(TURRET_GLOW_PIN, i);
    delay(TURRET_GLOW_FADE_SPEED);
  }
}

// Takes yaw/pitch coordinates in turret reference frame, constrains them within turrent safe range, and aims at that position
inline void turret_go_to(int yaw, int pitch){
  Serial.print("Turret aim (");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.println(")");

  yaw = constrain(yaw, TURRET_YAW_MIN, TURRET_YAW_MAX);
  pitch = constrain(pitch, TURRET_PITCH_MIN, TURRET_PITCH_MAX);

  Serial.print("Turret at (");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.println(")");
  
  turretYawServo.write(yaw);
  turretPitchServo.write(pitch);
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
#define GIMBAL_YAW_PIN 12
#define GIMBAL_YAW_INVERTED
#define GIMBAL_PITCH_PIN 8
#define GIMBAL_PITCH_INVERTED
Servo gimbalYawServo;
Servo gimbalPitchServo;

// Head laser gimbal constants
#define GIMBAL_YAW_CENTER 91
const unsigned short GIMBAL_YAW_MIN = GIMBAL_YAW_CENTER - 45;//#define GIMBAL_YAW_MIN 45
const unsigned short GIMBAL_YAW_MAX = GIMBAL_YAW_CENTER + 45;//#define GIMBAL_YAW_MAX 135
#define GIMBAL_PITCH_CENTER 102
const unsigned short GIMBAL_PITCH_MIN = GIMBAL_YAW_CENTER - 45;//#define GIMBAL_PITCH_MIN 45
const unsigned short GIMBAL_PITCH_MAX = GIMBAL_YAW_CENTER + 45;//#define GIMBAL_PITCH_MAX 135 

void setupGimbal(){
  // First TURN OFF THE LASER!
  pinMode(GIMBAL_LASER_PIN, OUTPUT);
  digitalWrite(GIMBAL_LASER_PIN, LOW);

  Serial.println("SETUP LASER GIMBAL");

  gimbalYawServo.attach(GIMBAL_YAW_PIN);
  gimbalYawServo.write(90);
  gimbalPitchServo.attach(GIMBAL_PITCH_PIN);
  gimbalPitchServo.write(90);
}

void gimbal_laser_on(){
  Serial.println("Gimbal laser ON!");
  digitalWrite(GIMBAL_LASER_PIN, HIGH);
}

void gimbal_laser_off(){
  Serial.println("Gimbal laser OFF.");
  digitalWrite(GIMBAL_LASER_PIN, LOW);
}

// Takes yaw/pitch coordinates in gimbal reference frame, constrains them within turrent safe range, and aims at that position
inline void gimbal_go_to(int yaw, int pitch){
  Serial.print("Gimbal aim (");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.println(")");

  yaw = constrain(yaw, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
  pitch = constrain(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

  Serial.print("Gimbal at (");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.println(")");
  
  gimbalYawServo.write(yaw);
  gimbalPitchServo.write(pitch);
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
  Serial.println("Parsing turret command...");

  // Fire command
  if (Serial.peek() == TURRET_FIRE_MESSAGE_INDICTOR){
    Serial.println("Turret FIRE message.");
    Serial.read(); // clear the turret fire message indicator
    turret_fire();

  // Aim command  
  }else if (Serial.peek() == TURRET_AIM_MESSAGE_INDICATOR){
    Serial.println("Turret AIM message...");
    Serial.read(); // clear the turret aim message indicator
    while (Serial.available() < (COORDINATE_MESSAGE_DATA_LENGTH)){
      delay(1); // wait for new all the data to come in
    }
    
    short turretYaw, turretPitch;
    parseCoordinates(&turretYaw, &turretPitch);
    turret_aim(turretYaw, turretPitch);  
    
  } // Possible other types of turret messages
}

inline void parseGimbalCommand(){
  Serial.read(); // clear the laser command frame start byte
  numBytesAvailable = Serial.available();
  Serial.println("Parsing laser command...");

  // laser ON command
  if (Serial.peek() == GIMBAL_LASERON_MESSAGE_INDICTOR){
    Serial.println("Gimbal Laser ON message.");
    Serial.read(); // clear the laser on message indicator
    gimbal_laser_on();

  // laser OFF
  }else if (Serial.peek() == GIMBAL_LASERON_MESSAGE_INDICTOR){
    Serial.println("Gimbal Laser OFF message.");
    Serial.read(); // clear the laser off message indicator
    gimbal_laser_off();
    
  // Aim command  
  }else if (Serial.peek() == GIMBAL_AIM_MESSAGE_INDICATOR){
    Serial.println("Gimbal AIM message...");
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
  
  Serial.println("TURRET ACTIVATED");
  turret_aim(0,0);
  turret_fade_in();

  #ifdef potPin
    pinMode(potPin, INPUT);
    gimbal_laser_on();
    turret_laser_on();
  #endif

  //testGimbal();
  //delay(2000);
  //testTurret();
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
