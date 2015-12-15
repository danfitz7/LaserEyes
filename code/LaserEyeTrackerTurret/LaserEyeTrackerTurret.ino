/* Arduino code for MAS.S65 Science Fiction Fabrication Class, Fall 2015
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


#define potPin A0

// SHOULDER TURRET PINS
#define TURRET_GUN_PIN 2  // RED
#define TURRET_GLOW_PIN 3 //Blue
#define TURRET_LASER_PIN 4//Green
#define TURRET_YAW_PIN 6  //Orange
#define TURRET_PITCH_PIN 5//Yellow
Servo turretYawServo;
Servo turretPitchServo;

// SHOULDER TURRET CONSTANTS
#define TURRET_YAW_MIN 0
#define TURRET_YAW_CENTER 97
#define TURRET_YAW_MAX 140
#define TURRET_PITCH_MIN 40
#define TURRET_PITCH_CENTER 95
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
void fire(){
  Serial.println("FIRE!");
  
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
void turret_aim(int yaw, int pitch){
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
void turret_aim_calibrated(int yaw, int pitch){
  yaw += TURRET_YAW_CENTER;
  pitch += TURRET_PITCH_CENTER;

  turret_aim(yaw, pitch);
}

void testTurret(){
  Serial.println("TEST TURRET");
  
  turret_fade_in();

  turret_laser_off();
  delay(1000);
  turret_laser_on();
  
  #define turret_wait_time 2000
  turret_aim(45,90);
  delay(turret_wait_time);
  turret_aim(45, 135);
  delay(turret_wait_time);
  turret_aim(90, 90);
  delay(turret_wait_time);
  turret_aim(90, 45);
  delay(turret_wait_time);
  turret_aim(90, 135);
  delay(turret_wait_time);
  turret_aim(90,90);
  delay(turret_wait_time);

  fire();

  turret_fade_out();
}

void setup() {
  Serial.begin(115200);
  setupTurret();
  
  Serial.println("TURRET ACTIVATED");
  turret_aim_calibrated(0,0);
  turret_fade_in();
}

// Serial communication
#define TURRET_COMMAND_FRAME_START 'T'
#define TURRET_FIRE_MESSAGE_INDICTOR 'F'
#define TURRET_AIM_MESSAGE_INDICATOR 'A'
#define TURRET_AIM_MESSAGE_DATA_LENGTH 4
int numBytesAvailable = 0;

// Packs the next 4 bytes in the Serial buffer in to 2 shorts (2-bytes each)
// WARNING: This should only be called if there are actually at-least 4 bytes in the buffer (Serial.available() >= 4 has just been verified)
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

void loop() {

  // If there are more than 3 bytes available
  numBytesAvailable = Serial.available();
  if (numBytesAvailable >= 1){

    // if we're receiveing a turret command
    if (Serial.peek() == TURRET_COMMAND_FRAME_START){
      serial.read(); // clear the turret command frame start
      numBytesAvailable = Serial.available();

      // Fire command
      if (Serial.peek() == TURRET_FIRE_MESSAGE_INDICTOR && numBytesAvailable >= 1){
        Serial.println("TURRET FIRE COMMAND");
        Serial.read(); // Read the TURRET_FIRE_MESSAGE_INDICTOR
        turret_fire();

      // Aim command  
      }else if (Serial.peek() == TURRET_AIM_MESSAGE_INDICATOR && numBytesAvailable >= TURRET_AIM_MESSAGE_DATA_LENGTH){
        Serial.println("TURRET AIM COMMAND");

        short turretYaw, turretPitch;
        parseCoordinates(&turretYaw, &turretPitch);
        turret_aim_calibrated(turretYaw, turretPitch);  
          
      } // Possible other types of commands
      
    }// else if (Serial.peek() == LASER_COMMAND_FRAME_START
     // TODO: laser command parsing here


    
  } // end check available bytes
}
