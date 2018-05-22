/*
 * Gimbal Board Software
 * Rev 1, 2018
 * Used with 2 Drill Board Rev 1 boosters on a single TIVA
 * Andrew Van Horn
 * 
 * Controls 2 open-loop motors: Pan, Tilt
 *          1 closed loop motor with two limit switches for position limits and one encoder: Mast (Up/Down)
 *          3 Servos :Roll, Camera Zoom, Camera Focus 
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
const uint8_t MAST_ENCODER_PIN             = PD_0;  //Using X9 Encoder 1

//////////////////////////////////////////
//Roll Servo runs off X7 Limit Switch 1
const uint8_t ROLL_SERVO_PIN = PK_1;

/////////////////////////////////////////////////////////////////////////////
const uint8_t CAMERA_ZOOM_PIN  = PK_0; //Camera Zoom  runs off X7 Limit Switch 2
const uint8_t CAMERA_FOCUS_PIN = PB_5; //Camera Focus runs off X7 Limit Switch 3

//Mast Encoder Setup///////////////////////
const uint16_t MAST_UP_SERVO_VALUE    = 1;
const uint16_t MAST_DOWN_SERVO_VALUE  = 1;
const int      MAST_CLOSED_LOOP_SPEED = 300; 

uint16_t       mast_move_to_position  = 0;

////////////////////
// RoveComm Setup //
////////////////////
//RoveComm Instantiations/////
RoveCommEthernetUdp  RoveComm;
RoveWatchdog         Watchdog;

//////////////////////
RoveVnh5019 PanMotor;
RoveVnh5019 TiltMotor;
RoveVnh5019 MastMotor;

Servo       RollServo;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// notice in https://github.com/energia/Energia/blob/master/hardware/lm4f/libraries/Servo/Servo.h 
// #define MIN_SERVO_PULSE_WIDTH       544
// #define DEFAULT_SERVO_PULSE_WIDTH   1500 
// #define MAX_SERVO_PULSE_WIDTH       2400 
// unsigned int attach(unsigned int pin, int min = MIN_SERVO_PULSE_WIDTH, int max = MAX_SERVO_PULSE_WIDTH);

// effectively, the cable is implementing these as a 'continuous servo' which looks like 1000:1500:2000 in micros for reverse_speed:stop:forward_speed_) instead of a 'position servo' (0 to 180 position degrees)

#define RC_CAMERA_MAX_REVERSE    1000
#define RC_CAMERA_ZERO           1500
#define RC_CAMERA_MAX_FORWARD    2000 

Servo       CameraZoom;
Servo       CameraFocus;

void estop(); // Watchdog Estop Function

////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  RoveComm.begin(ROVE_FIRST_OCTET, ROVE_SECOND_OCTET, ROVE_THIRD_OCTET, GIMBALBOARD_FOURTH_OCTET);
  delay(1);
  
  Serial.begin(9600);
  delay(1);
  
  PanMotor.begin( PAN_INA_PIN,  PAN_INB_PIN,  PAN_PWM_PIN);
  TiltMotor.begin(TILT_INA_PIN, TILT_INB_PIN, TILT_PWM_PIN);   
  MastMotor.begin(MAST_INA_PIN, MAST_INB_PIN, MAST_PWM_PIN);  
  
  RollServo.attach(  ROLL_SERVO_PIN);
  CameraZoom.attach( CAMERA_ZOOM_PIN,  RC_CAMERA_MAX_REVERSE, RC_CAMERA_MAX_FORWARD); 
  CameraFocus.attach(CAMERA_FOCUS_PIN, RC_CAMERA_MAX_REVERSE, RC_CAMERA_MAX_FORWARD); 
  delay(10);
  
  // I'm assuming they mount the servo centered at 90 (so that it sweep -90 degrees clockwise to 0 position, and +90 degrees to counter clockwise 180 position)
  RollServo.write(90);
  CameraZoom.writeMicroseconds( RC_CAMERA_ZERO); 
  CameraFocus.writeMicroseconds(RC_CAMERA_ZERO);
  
  pinMode(MAST_ENCODER_PIN, INPUT);
  delay(10);
   
  Watchdog.begin(estop, 500);
}

///////////////////////////////////////////////////////////////////
void loop()
{   
  uint16_t data_id; 
  size_t   data_size; 
  int8_t   data[2];

  RoveComm.read(&data_id, &data_size, data);

  switch(data_id)
  {
    case GIMBAL_PAN:
    {
      int pan_speed = *(int16_t*)(data); 
      pan_speed  = map(pan_speed,  RED_MAX_REVERSE, RED_MAX_FORWARD, VNH5019_MAX_REVERSE, VNH5019_MAX_FORWARD);       
      PanMotor.drive(pan_speed);       
      Watchdog.clear();
      break;
    }

    case GIMBAL_TILT:
    {
      int tilt_speed = *(int16_t*)(data); 
      tilt_speed  = map(tilt_speed,  RED_MAX_REVERSE, RED_MAX_FORWARD, VNH5019_MAX_REVERSE, VNH5019_MAX_FORWARD);      
      TiltMotor.drive(tilt_speed);       
      Watchdog.clear();
      break;
    }

    case GIMBAL_ROLL:
    {
      // I don't see any reason to special case a continuous position command from RED, I would advise Skelton uses -1000 for full clockwise rotation and 1000 for full counter clockwise
      int servo_position = *(int16_t*)(data); 
      servo_position  = map(servo_position,  RED_MAX_REVERSE, RED_MAX_FORWARD, 0, 180 );  	  
      RollServo.write(servo_position);       
      Watchdog.clear();
      break;
    }

    case CAMERA1_COMMAND:
    {
      // I don't see any reason to special case a continuous zoom command from RED, I would advise Skelton uses -1000 for full zoom/focus out and 1000 for full zoom/focus in
      int camera_command_data = *(int16_t*)(data);  
	  camera_command_data  = map(camera_command_data,  RED_MAX_REVERSE, RED_MAX_FORWARD, RC_CAMERA_MAX_REVERSE, RC_CAMERA_MAX_FORWARD);  
      switch(camera_command_data)
      {
        case CAMERA_STOP:
        {			
          CameraZoom.write(camera_command_data);
          break;
        }
  
        case CAMERA_ZOOM_IN:
        {
          CameraZoom.write(camera_command_data);
          break;
        }

        case CAMERA_ZOOM_OUT:
        {
          CameraZoom.write(camera_command_data);
          break;
        }
        
        case CAMERA_FOCUS_IN:
        {
          CameraFocus.write(camera_command_data);
          break;
        }
          
        case CAMERA_FOCUS_OUT:
        {
          CameraFocus.write(camera_command_data);
          break;
        }
        Watchdog.clear(); 
		
		default: // always add a default
		  break;
	  }
    }  
    
    case MAST_MOVE_OPEN_LOOP:
    {
      //if( Todo?...
      int mast_speed = *(int16_t*)(data);       
      MastMotor.drive(mast_speed);  
      mast_move_to_position = 0;     
      Watchdog.clear();
      break;
    }

   case MAST_MOVE_TO_POSITION: // Todo once I have a spec
    {    
      mast_move_to_position = data+1
      Watchdog.clear();
      break;
    }
    
    default:
      break;
  }//End Switch Data ID

  ////////////////////////////////////////
  //     Mast Closed Loop Functions     //
  ////////////////////////////////////////
  
  // This part is what we should talk through, I wasn't aware we get encoders?

  if(  (analogRead(MAST_ENCODER_PIN) >= MAST_UP_SERVO_VALUE  ) 
    || (analogRead(MAST_ENCODER_PIN) <= MAST_DOWN_SERVO_VALUE)) // up to... but I like prettyfying my || and &&
  {
	MastMotor.brake(0);  // I'm still pushing for brackets on ALL if's....
	
  } else if(mast_move_to_position) // I think you would have immediately started driving even after you braked?
  {
    if(mast_move_to_position = 1) 
    {
      MastMotor.drive(-MAST_CLOSED_LOOP_SPEED);
	}
	
    if(mast_move_to_position = 2) 
	{  
      MastMotor.drive( MAST_CLOSED_LOOP_SPEED);
	}
  }
}

//////////////////////////
void estop()
{
  PanMotor.brake(0);  
  TiltMotor.brake(0);
  MastMotor.brake(0); 
  return;     
}

