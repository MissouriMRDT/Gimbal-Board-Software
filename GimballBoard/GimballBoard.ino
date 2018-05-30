/*
 * Gimbal Board Software
 * Rev 1, 2018
 * Used with 2 Drill Board Rev 1 boosters on a single TIVA
 * Andrew Van Horn
 * 
 * Controls 2 open-loop motors Pan, Tilt), 1 closed loop motor with two limit switches for position and an encoder (Mast up/down) 
 *          1 servo (roll), Camera Zoom (Hardware PWM write from a limit switch channel), Camera Focus (Hardware PEM write from a limit switch channel)
 */

#include "RoveWare.h"
#include "Servo.h"

////////////////////
//   Board Pins   //
////////////////////
/////////////////////////////////////////////
// Pan Motor pins drive Motor 1 on header X7

const uint8_t PAN_INA_PIN    = PH_0;
const uint8_t PAN_INB_PIN    = PP_5;
const uint8_t PAN_PWM_PIN    = PG_1;

/////////////////////////////////////////////////
// Tilt Motor pins drive Motor 2 on header X7

const uint8_t TILT_INA_PIN       = PH_1;
const uint8_t TILT_INB_PIN       = PA_7;
const uint8_t TILT_PWM_PIN       = PK_4;

//////////////////////////////////////////////////////////////////////////////////
// Mast Motor pins drive Motor 1 on header X9

const uint8_t MAST_INA_PIN           = PL_0;
const uint8_t MAST_INB_PIN           = PH_2;
const uint8_t MAST_PWM_PIN           = PF_1;

const uint8_t MAST_TOP_LIMIT_SWITCH_PIN    = PE_3;  //Using X9 Limit Switch 1
const uint8_t MAST_BOTTOM_LIMIT_SWITCH_PIN = PE_2;  //Using X9 Limit Switch 2

//////////////////////////////////////////
//Roll Servo runs off X7 LS 1
const uint8_t ROLL_SERVO_PIN = PK_0;
//////////////////////////////////////PK_0///////////////////////////////////////
const uint8_t CAMERA_ON_OFF_PIN  = PK_0; //Camera On Off runs off X7 Limit Switch 2
const uint8_t CAMERA_RECORD_PIN  = PB_5; //Camera Record runs off X7 Limit Switch 3
const uint8_t CAMERA_ZOOM_PIN    = PB_4; //Camera Zoom  runs off X7 Limit Switch 4
const uint8_t CAMERA_FOCUS_PIN   = PB_5; //Camera Focus  runs off X7 Limit Switch 3


//Zoom Setup//////////////////////////
const int RC_CAMERA_MAX_REVERSE = 0;
const int RC_CAMERA_ZERO        = 90;
const int RC_CAMERA_MAX_FORWARD = 180;

//Mast Encoder Setup///////////////////////
const uint16_t MAST_UP_SERVO_VALUE    = 1;
const uint16_t MAST_DOWN_SERVO_VALUE  = 1;
const int      MAST_CLOSED_LOOP_SPEED = 300; 
uint8_t        mast_move_to_position  = 0;
bool           mast_going_up;
int16_t        mast_speed;

////////////////////////////////
int16_t roll_servo_position = 0;

//Map Values//////////////////////
const int PAN_MAX_FORWARD  =  250;
const int PAN_MAX_REVERSE  = -250;

const int TILT_MAX_FORWARD =  250;
const int TILT_MAX_REVERSE = -250;

const int MAST_MAX_FORWARD =  250;
const int MAST_MAX_REVERSE = -250;

////////////////////
// RoveComm Setup //
////////////////////

//Read Variables////
uint16_t data_id; 
size_t   data_size; 
uint8_t  data[10];

//////////////////////
RoveVnh5019 PanMotor;
RoveVnh5019 TiltMotor;
RoveVnh5019 MastMotor;
Servo       RollServo;
Servo       CameraOnOff;
Servo       CameraRecord;
Servo       CameraZoom;
Servo       CameraFocus;


void estop(); // Watchdog Estop Function
void generateCameraSignal(int amt, int pin);

RoveWatchdog Watchdog;

////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  roveComm_Begin(ROVE_FIRST_OCTET, ROVE_SECOND_OCTET, ROVE_THIRD_OCTET, GIMBALBOARD_FOURTH_OCTET);
  delay(1);

  Serial.begin(9600);
  delay(1);

  pinMode(ROLL_SERVO_PIN, OUTPUT);
  
  PanMotor.begin( PAN_INA_PIN,  PAN_INB_PIN,  PAN_PWM_PIN);
  TiltMotor.begin(TILT_INA_PIN, TILT_INB_PIN, TILT_PWM_PIN);   
  MastMotor.begin(MAST_INA_PIN, MAST_INB_PIN, MAST_PWM_PIN);  
  
  RollServo.attach(  ROLL_SERVO_PIN);
  CameraZoom.attach( CAMERA_ZOOM_PIN);
  CameraOnOff.attach(CAMERA_ON_OFF_PIN);
  CameraRecord.attach(CAMERA_RECORD_PIN);
  
  delay(10);
  RollServo.write(90);
  
  CameraZoom.write(RC_CAMERA_ZERO);
  CameraOnOff.write(RC_CAMERA_ZERO);
  CameraRecord.write(RC_CAMERA_ZERO);
  
  pinMode(CAMERA_ZOOM_PIN,  OUTPUT);
  pinMode(CAMERA_FOCUS_PIN, OUTPUT);

  digitalWrite(CAMERA_ZOOM_PIN,  0); 
  digitalWrite(CAMERA_FOCUS_PIN, 0);
  
  pinMode(MAST_TOP_LIMIT_SWITCH_PIN,    INPUT);
  pinMode(MAST_BOTTOM_LIMIT_SWITCH_PIN, INPUT);

  pinMode(ROLL_SERVO_PIN, OUTPUT);
  
  delay(10);
   
  Watchdog.begin(estop, 500);
}

///////////////////////////////////////////////////////////////////
void loop()
{   
  roveComm_GetMsg(&data_id, &data_size, data);
  Watchdog.clear();

 Serial.println("");
  Serial.print("ID: ");
  Serial.println(data_id);
  Serial.print("Data: ");
  Serial.println(*(int16_t*)data);
  delay(10);
   
  switch(data_id)
  {
    case GIMBAL_PAN_TILT_ROLL_MAST_ZOOM_OPEN_LOOP:
    {
      Watchdog.clear();

      int16_t *gimbal_values = ((int16_t*)(data));
      
      int16_t pan_speed   = (gimbal_values[0]);
      int16_t tilt_speed  = (gimbal_values[1]);
      int16_t roll_inc    = (gimbal_values[2]);
              mast_speed  = (gimbal_values[3]);
      int16_t zoom_speed  = (gimbal_values[4]);
      int16_t focus_speed = (gimbal_values[5]);

      delay(10);
      Serial.print("Pan: ");
      Serial.println(pan_speed);
      Serial.print("Tilt: ");
      Serial.println(tilt_speed);
      Serial.print("Roll: ");
      Serial.println(roll_inc);
      Serial.print("Mast: ");
      Serial.println(mast_speed);
      Serial.print("Zoom: ");
      Serial.println(zoom_speed);
      
      pan_speed = map(pan_speed, RED_MAX_REVERSE, RED_MAX_FORWARD, PAN_MAX_REVERSE, PAN_MAX_FORWARD); 
      tilt_speed = map(tilt_speed, RED_MAX_REVERSE, RED_MAX_FORWARD, TILT_MAX_REVERSE, TILT_MAX_FORWARD);  
      roll_servo_position += roll_inc; 
      mast_speed = map(mast_speed, RED_MAX_REVERSE, RED_MAX_FORWARD, MAST_MAX_REVERSE, MAST_MAX_FORWARD);  
      zoom_speed = map(zoom_speed, RED_MAX_REVERSE, RED_MAX_FORWARD, RC_CAMERA_MAX_REVERSE, RC_CAMERA_MAX_FORWARD);  

      Serial.println("");
      Serial.print("Pan: ");
      Serial.println(pan_speed);
      Serial.print("Tilt: ");
      Serial.println(tilt_speed);
      Serial.print("Roll: ");
      Serial.println(roll_inc);
      Serial.print("Mast: ");
      Serial.println(mast_speed);
      Serial.print("Zoom: ");
      Serial.println(zoom_speed);
      
      PanMotor.drive(pan_speed);       
      TiltMotor.drive(tilt_speed); 
   //   CameraOnOff.write(roll_servo_position); 
      if(!((mast_speed > 0) && !digitalRead(MAST_TOP_LIMIT_SWITCH_PIN)))
      {
        MastMotor.drive(mast_speed);
        Serial.println("Mast Moving");
      }
      
      CameraZoom.write(zoom_speed);

      Serial.println(mast_speed);
      break;
    }

    default:
      break;
  }//End Switch Data ID
  
if((mast_speed > 0) && !digitalRead(MAST_TOP_LIMIT_SWITCH_PIN))
{
  MastMotor.brake(mast_speed);
  Serial.println("Mast Brake");
}

if(digitalRead(MAST_TOP_LIMIT_SWITCH_PIN))
{
  roll_servo_position = 90;
  Serial.println("SERVO");
 // RollServo.write(roll_servo_position);
}
  
  
}

//////////////////////////
void estop()
{
  PanMotor.brake(0);  
  TiltMotor.brake(0);
  MastMotor.brake(0); 
  Serial.println("WatchDog Triggered");
  return;     
}

//////////////////////////////////////////
void generateCameraSignal(int amt, int pin)
{
  for(int i = 0; i<5l; i++)
  {
    digitalWrite(pin, HIGH);
    delayMicroseconds(amt);
    digitalWrite(pin, LOW);
    delayMicroseconds(20000-amt);
  }
  return;
}

