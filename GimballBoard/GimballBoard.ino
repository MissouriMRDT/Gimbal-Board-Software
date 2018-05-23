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
//Roll Servo runs off X7 Limit Switch 1
const uint8_t ROLL_SERVO_PIN = PK_1;

/////////////////////////////////////////////////////////////////////////////
const uint8_t CAMERA_ZOOM_PIN  = PK_0; //Camera Zoom  runs off X7 Limit Switch 2
const uint8_t CAMERA_FOCUS_PIN = PB_5; //Camera Focus runs off X7 Limit Switch 3

//Zoom Setup//////////////////////////
const int CAMERA_SHORT_SIGNAL  = 1000;
const int CAMERA_MIDDLE_SIGNAL = 1500;
const int CAMERA_LONG_SIGNAL   = 2000;

//Mast Encoder Setup///////////////////////
const uint16_t MAST_UP_SERVO_VALUE    = 1;
const uint16_t MAST_DOWN_SERVO_VALUE  = 1;
const int      MAST_CLOSED_LOOP_SPEED = 300; 
uint8_t        mast_move_to_position  = 0;
bool           mast_going_up;

////////////////////
// RoveComm Setup //
////////////////////
//RoveComm Instantiations/////
RoveCommEthernetUdp  RoveComm;
RoveWatchdog         Watchdog;

//Read Variables////
uint16_t data_id; 
size_t   data_size; 
uint8_t  data[2];

//////////////////////
RoveVnh5019 PanMotor;
RoveVnh5019 TiltMotor;
RoveVnh5019 MastMotor;
Servo       RollServo;

void estop(); // Watchdog Estop Function
void generateCameraSignal(int amt, int pin);

////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  RoveComm.begin(ROVE_FIRST_OCTET, ROVE_SECOND_OCTET, ROVE_THIRD_OCTET, GIMBALBOARD_FOURTH_OCTET);
  delay(1);
  
 // Serial.begin(9600);
  delay(1);
  
  PanMotor.begin( PAN_INA_PIN,  PAN_INB_PIN,  PAN_PWM_PIN);
  TiltMotor.begin(TILT_INA_PIN, TILT_INB_PIN, TILT_PWM_PIN);   
  MastMotor.begin(MAST_INA_PIN, MAST_INB_PIN, MAST_PWM_PIN);  
  
  RollServo.attach(ROLL_SERVO_PIN);
  delay(10);
  RollServo.write(90);
  pinMode(CAMERA_ZOOM_PIN,  OUTPUT);
  pinMode(CAMERA_FOCUS_PIN, OUTPUT);

  digitalWrite(CAMERA_ZOOM_PIN,  0); 
  digitalWrite(CAMERA_FOCUS_PIN, 0);
  
  pinMode(MAST_TOP_LIMIT_SWITCH_PIN,    INPUT);
  pinMode(MAST_BOTTOM_LIMIT_SWITCH_PIN, INPUT);
  delay(10);
   
  Watchdog.begin(estop, 500);
}

///////////////////////////////////////////////////////////////////
void loop()
{   
  RoveComm.read(&data_id, &data_size, data);

  switch(data_id)
  {
    case GIMBAL_PAN:
    {
      int pan_speed = *(int16_t*)(data);       
      PanMotor.drive(pan_speed);       
      Watchdog.clear();
      break;
    }

    case GIMBAL_TILT:
    {
      int tilt_speed = *(int16_t*)(data);       
      TiltMotor.drive(tilt_speed);       
      Watchdog.clear();
      break;
    }

    case GIMBAL_ROLL:
    {
      int servo_position = data[0];       
      RollServo.write(servo_position);       
      Watchdog.clear();
      break;
    }

    case CAMERA1_COMMAND:
    {
      int command = data[0];      
      switch(command)
        {
          case CAMERA_STOP:
          {
            generateCameraSignal(CAMERA_MIDDLE_SIGNAL, CAMERA_ZOOM_PIN);
            generateCameraSignal(CAMERA_MIDDLE_SIGNAL, CAMERA_FOCUS_PIN);
            break;
          }
  
          case CAMERA_ZOOM_IN:
          {
            generateCameraSignal(CAMERA_LONG_SIGNAL, CAMERA_ZOOM_PIN);
            break;
          }

          case CAMERA_ZOOM_OUT:
          {
            generateCameraSignal(CAMERA_SHORT_SIGNAL, CAMERA_ZOOM_PIN);
            break;
          }
        
          case CAMERA_FOCUS_IN:
          {
            generateCameraSignal(CAMERA_LONG_SIGNAL, CAMERA_ZOOM_PIN);
            break;
          }
          
          case CAMERA_FOCUS_OUT:
          {
            generateCameraSignal(CAMERA_SHORT_SIGNAL, CAMERA_ZOOM_PIN);
            break;
          }
        }//End Case Camera Command
        Watchdog.clear(); 
    }  
    
    case MAST_MOVE_OPEN_LOOP:
    {
      Watchdog.clear();
      mast_move_to_position = 0; 
      int mast_speed = *(int16_t*)(data);  
      mast_going_up = (mast_speed == abs(mast_speed));
      if(digitalRead(MAST_TOP_LIMIT_SWITCH_PIN)    && mast_going_up)
      {
        MastMotor.brake(0);
        break;  
      }
      else if(digitalRead(MAST_BOTTOM_LIMIT_SWITCH_PIN) && !mast_going_up)
      {
        MastMotor.brake(0);
        break;  
        
      }
      
      MastMotor.drive(mast_speed);  
      break;
    }

   case MAST_MOVE_TO_POSITION:
    {    
      mast_move_to_position = data[0]+1; //I asked Skelton to send 0 for position down, 1 for position up
      Watchdog.clear();
      break;
    }
    
    default:
      break;
  }//End Switch Data ID

  ////////////////////////////////////////
  //        Mast   Functions            //
  ////////////////////////////////////////
  Serial.println(mast_move_to_position);
  
  if(mast_move_to_position == 1)
  {
    if(digitalRead(MAST_BOTTOM_LIMIT_SWITCH_PIN))
    {
      mast_move_to_position = 0;
    }
    else
    {
      MastMotor.drive(-MAST_CLOSED_LOOP_SPEED);
      Serial.println("Going DOWN");
     mast_going_up = 0;
    }
  }
  else if(mast_move_to_position == 2) 
  {
    if(digitalRead(MAST_TOP_LIMIT_SWITCH_PIN))
    {
      mast_move_to_position = 0;
    }
    else
    {
      MastMotor.drive( MAST_CLOSED_LOOP_SPEED);
      Serial.println("Going UP");
      mast_going_up = 1;
    }
    
  }
  
  if(digitalRead(MAST_TOP_LIMIT_SWITCH_PIN)    && mast_going_up)
  {
    MastMotor.brake(0); 
  }
  else if(digitalRead(MAST_BOTTOM_LIMIT_SWITCH_PIN) && !mast_going_up)
  {
    MastMotor.brake(0);   
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

