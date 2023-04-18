///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            DIFFERENTIAL ROBOT FIRMWARE                                            //
//                                      DEVELOPED AT THE UNIVERSITY OF ALCALÁ                                        //
// You can find more information at: www.hindawi.com/journals/js/2019/8269256/?utm_medium=author&utm_source=Hindawi  //
//                                              Please include reference                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
//SRE_LOLA

#include "lola_board.h"
#include <PID_v1.h>
#include <math.h>

// Define a enum with the actions of Differential Robot
typedef enum action{
  FORWARD =    0,
  BACKWARD =   1,
  TURN_LEFT =  2,
  TURN_RIGHT = 3
}action_t;

// Define a enum with the possible errors
typedef enum error{
  NO_ERROR =            0,
  NO_NUMBER =           1,  // Waiting for a number, time-out
  OUT_RANGE =           2,  // Received number out of range
  SPEED_OUT_RANGE =     3,  // Received speed out of range
  RR_OUT_RANGE =        4,  // Radio Ratio out of range
  NO_AVAILABLE =        5,  // Received Command non available
  INERTIA_LIMIT_ERROR = 6,  // Distance lower than inertia limit
  ERROR_CODE =          7   // Code of Error
}error_t;

// Define a enum with the states of robot
typedef enum state{
  MOVE = 0,
  STOP = 1,
  RESET = 2  
} state_t;

// Define the struct with the data from Right Motor
typedef struct{
  int PPR = 713;                          // Pulses Per Revolution of Wheel
  float max_velocity = 240.0;             // Max velocity in RPM
  unsigned long encoderDER = 0;           // Incremental value Encoder 
  long encoderAbs = 0;                    // Absolute counter encoder value
  unsigned long last_value_encoder = 0;   // Save the last value lecture of incremental encoder
  uint8_t max_speed_pwm = 127;            // Max speed for PWM with our encode 0..127
  float duty_cycle_top_speed = 0.8;       // Duty for top speed for PWM
  long delta_encoder = 0;                 // Delta of pulses in sample time  
}right_motor_t;

// Define the struct with the data from Left Motor
typedef struct{
  int PPR = 713;                          // Pulses Per Revolution of Wheel
  float max_velocity = 240.0;             // Max velocity in RPM
  unsigned long encoderIZQ = 0;           // Incremental value Encoder
  long encoderAbs = 0;                    // Absolute counter encoder value
  unsigned long last_value_encoder = 0;   // Save the last value lecture of incremental encoder
  uint8_t max_speed_pwm = 127;            // Max speed for PWM with our encode 0..127
  float duty_cycle_top_speed = 0.8;       // Duty for top speed for PWM
  long delta_encoder = 0;                 // Delta of pulses in sample time  
}left_motor_t;

// Define the struct with the data from PID for Right Motor
typedef struct{
  // Input: is the real value
  // SetPoint: is de reference value
  // Output: is the control value
  double Input, SetPoint, Output;
  double Kp=0.7, Ki=0.5, Kd=0.7;
  // Limits of saturation
  double top_limit_sat = 255.0;
  double bottom_limit_sat = 0.0;
  // Sample time
  int ts = 100; // in ms
}PID_right_motor_t;

// Define the struct with the data from PID for Left Motor
typedef struct{
  // Input: is the real value
  // SetPoint: is de reference value
  // Output: is the control value  
  double Input, SetPoint, Output;
  double Kp=0.7, Ki=0.5, Kd=0.7;
  // Limits of saturation
  double top_limit_sat = 255.0;
  double bottom_limit_sat = 0.0;
  // Sample time
  int ts = 100; // in ms  
}PID_left_motor_t;

// Create the variables 
action_t sel_action;

error_t type_error;

state_t state;

// Variables with the mechanical data motors
right_motor_t MOT_R;
left_motor_t MOT_L; 

// Variables with the data for PID controllers motors
PID_right_motor_t PID_MOT_R;
PID_left_motor_t PID_MOT_L;

PID PID_R(&PID_MOT_R.Input, &PID_MOT_R.Output, &PID_MOT_R.SetPoint, PID_MOT_R.Kp, PID_MOT_R.Ki, PID_MOT_R.Kd, DIRECT);
PID PID_L(&PID_MOT_L.Input, &PID_MOT_L.Output, &PID_MOT_L.SetPoint, PID_MOT_L.Kp, PID_MOT_L.Ki, PID_MOT_L.Kd, DIRECT);

////////////////////
// FUNCION: SETUP //
////////////////////
void setup() 
{ 
  // Add the interrupt lines for encoders -> active for rising edge
  attachInterrupt(digitalPinToInterrupt(MOT_R_ENC_B_PIN), right_encoder_IRQHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(MOT_L_ENC_B_PIN), left_encoder_IRQHandler, RISING);

  // Set the mode of controllers PID
  PID_R.SetMode(AUTOMATIC);
  PID_L.SetMode(AUTOMATIC);

  // Put the sample time of PID
  PID_R.SetSampleTime(PID_MOT_R.ts);
  PID_L.SetSampleTime(PID_MOT_L.ts);

  // Put the limits of Saturation PID
  PID_R.SetOutputLimits(PID_MOT_R.bottom_limit_sat, PID_MOT_R.top_limit_sat);
  PID_L.SetOutputLimits(PID_MOT_L.bottom_limit_sat, PID_MOT_L.top_limit_sat);

  // Set encoder pins to inputs
  pinMode(MOT_L_ENC_B_PIN, INPUT);
  pinMode(MOT_R_ENC_B_PIN, INPUT);

  //Battery pin for voltaje measurement
  pinMode(BAT_PIN, INPUT);

  // Set all the motor control PWM pins outputs
  pinMode(MOT_R_PWM_PIN, OUTPUT);
  pinMode(MOT_L_PWM_PIN, OUTPUT);

  // Set all the motor control pins output for Enable and 
  // select the Turning Sense
  pinMode(MOT_R_A_PIN, OUTPUT);
  pinMode(MOT_R_B_PIN, OUTPUT);
  pinMode(MOT_L_A_PIN, OUTPUT);
  pinMode(MOT_L_B_PIN, OUTPUT);

  // Stop the motors writing a cero value in PWM
  analogWrite(MOT_R_PWM_PIN, 0);
  analogWrite(MOT_L_PWM_PIN, 0);

  // Init the serial port with a baudrate of 115200 baudios
  Serial.begin(115200);      
  Serial.println("LOLA INI ");

}  // End of setup()

///////////////////////////
// FUNCTION: void loop() //
///////////////////////////
void loop() 
{
  // put your main code here, to run repeatedly:
  state = RESET;
  Lola();
}  // end of loop()


//////////////////////////////////////////////////
// FUNCION: cuentaDER()
// Right encoder interrupt using only one pin
//////////////////////////////////////////////////
void right_encoder_IRQHandler(void)
{
  MOT_R.encoderDER++;    //Add one pulse
}  // end of cuentaDER

///////////////////////////////////////////////////
// FUNCION: cuentaIZQ()
//  Left encoder interrupt using only one pin
//////////////////////////////////////////////////
void left_encoder_IRQHandler(void)
{
  MOT_L.encoderIZQ++;  //Add one pulse
}  // end of cuentaIZQ


/////////////////////////////////////////////////////////////////
//  FUNCION: select_sense_turning_motors(action_t type_action) //
//  CONFIGURE THE PINS L298N FOR SELECT THE SENSE TURNING      //
//  dir_right (1: forward / 0: backwards)                      //
//  dir_left  (1: forward / 0: backwards)                      //
/////////////////////////////////////////////////////////////////
void select_sense_turning_motors(action_t type_action) 
{
  switch (type_action)
  {
    case FORWARD:
      // Sense turning for motor Right      
      digitalWrite(MOT_R_A_PIN, HIGH);
      digitalWrite(MOT_R_B_PIN, LOW);
      // Sense turning for motor Left
      digitalWrite(MOT_L_A_PIN, LOW);
      digitalWrite(MOT_L_B_PIN, HIGH);
    break;

    case BACKWARD:
      // Sense turning for motor Right
      digitalWrite(MOT_R_A_PIN, LOW);
      digitalWrite(MOT_R_B_PIN, HIGH);
      // Sense turning for motor Left
      digitalWrite(MOT_L_A_PIN, HIGH);
      digitalWrite(MOT_L_B_PIN, LOW);    
    break;

    case TURN_LEFT:
      // Sense turning for motor Right
      digitalWrite(MOT_R_A_PIN, HIGH);
      digitalWrite(MOT_R_B_PIN, LOW);
      // Sense turning for motor Left
      digitalWrite(MOT_L_A_PIN, HIGH);
      digitalWrite(MOT_L_B_PIN, LOW);
    break;

    case TURN_RIGHT:
      // Sense turning for motor Right
      digitalWrite(MOT_R_A_PIN, LOW);
      digitalWrite(MOT_R_B_PIN, HIGH);
      // Sense turning for motor Left
      digitalWrite(MOT_L_A_PIN, LOW);
      digitalWrite(MOT_L_B_PIN, HIGH);
    break;
  }  
}  // End of select_sense_turning_motors(action_t type_action)

////////////////////////////
// FUNCION: stop_motors() //
//      STOP_MOTORS       //
////////////////////////////
void stop_motors(void) 
{
  PORTB |= 0x3 << 3;                              
  PORTA &= 0xAA;
  //We will fix the same duty cycle for the PWM outputs to make the braking spped equal!
  analogWrite(MOT_R_PWM_PIN, 0);
  analogWrite(MOT_L_PWM_PIN, 0);
}  // End of stop_motors()

////////////////////////////////////////////////////
//        FUNCION: speed_normalization()          //
//                                                //
//  Speeds are normalized in order to work        //
//  at maximum speed give as MOT_X.max_speed_pwm  //
//                                                //
////////////////////////////////////////////////////
void speed_normalization(uint8_t velocity_right, uint8_t velocity_left)
{
  uint8_t vel_right=0, vel_left=0;

  // Normalization the speed for right motor in case the value PWM is upper to 127
  // in this comprobation the values between 255..128 are scaled to 0..127   
  if (velocity_right > MOT_R.max_speed_pwm)
    vel_right = abs(255 - velocity_right); 
  else
    vel_right = velocity_right;             
  
  // Normalization the speed for left motor in case the value PWM is upper to 127
  // in this comprobation the values between 255..128 are scaled to 0..127
  if (velocity_left > MOT_L.max_speed_pwm)
    vel_left = abs(255 - velocity_left);
  else
    vel_left = velocity_left;

  // Scale the values 0..127 to 0..(255 * Duty_cycle) for PWM where 127 -> (255 * Duty_cycle)
  int vel_right_scaled = int(((float) vel_right / (float) MOT_R.max_speed_pwm) * (255.0 * MOT_R.duty_cycle_top_speed));  
  int vel_left_scaled = int(((float) vel_left / (float) MOT_L.max_speed_pwm) * (255.0 * MOT_L.duty_cycle_top_speed)); 

  // Refresh the SetPoint of each motor for PID module
  PID_MOT_R.SetPoint = vel_right_scaled;
  PID_MOT_L.SetPoint = vel_left_scaled;

} // end of speed_normalization

//////////////////////////////////////////////////
// FUNCTION: CHECK_ACTION                       //
// With the velocities info check the action of //
// differential base robot                      //
//////////////////////////////////////////////////

action_t check_type_action(int velocity_right, int velocity_left)
{
  // Forward: both velocities is under tahn 127
  if (velocity_right < MOT_R.max_speed_pwm && velocity_left < MOT_L.max_speed_pwm)
    return FORWARD;
  // Backward: both velocities is upper than 127
  else if (velocity_right > MOT_R.max_speed_pwm && velocity_left > MOT_L.max_speed_pwm)
    return BACKWARD;
  // Turn left: right velocity is under than 127 and left velocity is upper than 127
  else if (velocity_right < MOT_R.max_speed_pwm && velocity_left > MOT_L.max_speed_pwm) 
    return TURN_LEFT;
  // Turn right: right velocity is upper tahn 127 and left velocity is under than 127
  else if (velocity_right > MOT_R.max_speed_pwm && velocity_left < MOT_L.max_speed_pwm)
    return TURN_RIGHT;         
}
///////////////////////////////////////////////////////////////
//  FUNCION:  READ_SPEED                                     //
//  Read a number from the serial port with <number> digits  //
///////////////////////////////////////////////////////////////
short int read_speed(int n_digits) 
{
  char speed[8];

  // Wait 5 ms to be sure the bytes have arrived
  delay(5);
  //  sprintf(speed,"%d",SERIA.available());
  //  SERIA.println(speed);
  if (digitalRead(SW5_PIN) == LOW) 
  {
    if (Serial2.available() > n_digits - 1)
    {
      Serial2.readBytes(speed, n_digits);
      speed[n_digits] = '\0';  // Append a NULL character to terminate the string! Not needed if initialized with char speed[5] = {0}...

      return (uint8_t) atoi(speed);
    }

    else
    {
      type_error = NO_NUMBER;
      return 0;
    }
  }

  else 
  {
    if (Serial.available() > n_digits - 1)
    {
      Serial.readBytes(speed, n_digits);
      speed[n_digits] = '\0';  // Append a NULL character to terminate the string! Not needed if initialized with char speed[5] = {0}...

      return (uint8_t) atoi(speed);
    }

    else
    {
      type_error = NO_NUMBER;
      return 0;
    }
  }
}  // End of read_speed()

///////////////////////////////////////////
// FUNCION:   ANALYZE_ORDER              //
// Parse command information and trigger //
// all the necessary functions           //
///////////////////////////////////////////
void analyze_order(char order) 
{

  switch (order)
  {
    case   'V':
      uint8_t velocity_right = read_speed(3);
      uint8_t velocity_left =  read_speed(3);

      // Check the value of velocity
      if (velocity_right == 0 && velocity_left ==0)
      {
        // Stop the robot
        state = STOP;          
      } 

      else
      {      
        // Refresh the state
        state = MOVE;

        // Check and refresh the global variable sel_action with the possible action from velocity received
        sel_action = check_type_action(velocity_right, velocity_left);

        // Update the pins control for L298N
        select_sense_turning_motors(sel_action); 

        // Normalize the velocities for wheels with the data received by Serial Port 
        // and update the SetPoint for PID controller      
        speed_normalization(velocity_right, velocity_left); 
      }
    break;

    // Stop motors and calcucompute the new positions
    case '?': // '?'
      stop_motors();
      state = RESET;
    break;

    // The error is sent back by adding 0x30 to the error code
    case 'E': // 'E'
      type_error = ERROR_CODE;
      
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.write(0x45);  // 'E'
        Serial2.write(0x30 + type_error);
        Serial2.println("");
      }
      else {
        Serial.write(0x45);  // 'E'
        Serial.write(0x30 + type_error);
        Serial.println("");
      }
    break;

    case 'Q': // 'Q'
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("\nQ:");
        Serial2.print(MOT_L.encoderIZQ);
        Serial2.print(" ");
        Serial2.println(MOT_R.encoderDER);
      }
      else {
        Serial.print("\nQ:");
        Serial.print(MOT_L.encoderIZQ);
        Serial.print(" ");
        Serial.println(MOT_R.encoderDER);
      }
      break;

    case 'N': //'N'    Datos de los encoders en valor absoluto
      send_data_encoder();
    break;

    default:
      type_error = NO_AVAILABLE;
  }
}  // End of analyze_order()

////////////////////////////////////////////////////////////
// FUNCION: void update_speeds_from_PID_controller(void)  //
// This function perform a PID for speeds control.        //
////////////////////////////////////////////////////////////
void update_speeds_from_PID_controller(void)
{
  // Update the absolute counter value from encoders, the function needs
  // the global variable sel_action.
  update_encoder_abs_value(sel_action);

  // Measure the real velocity from wheels and covert to value between 0..(255 * Duty_Cycle)
  //                            Transfrom the delta_pulses of encoder, in one sample time, to RPM of Wheel                  Scale the RPM to value of PWM 0..(255 * Duty_clycle)
  //                ---------------------------------------------------------------------------------------------------  -----------------------------------------------------------
  //                |                                                                                                 |  |                                                         |
  PID_MOT_R.Input = (float) MOT_R.delta_encoder * (1000.0 / (float) PID_MOT_R.ts) * (60.0) * (1.0 / (float) MOT_R.PPR) * (255.0 * MOT_R.duty_cycle_top_speed / MOT_R.max_velocity);
  PID_MOT_L.Input = (float) MOT_L.delta_encoder * (1000.0 / (float) PID_MOT_L.ts) * (60.0) * (1.0 / (float) MOT_L.PPR) * (255.0 * MOT_L.duty_cycle_top_speed / MOT_L.max_velocity);

  // Compute the PID controller
  PID_R.Compute();
  PID_L.Compute();

  // Write, as PWM duty cycles, the speeds for each wheel
  analogWrite(MOT_R_PWM_PIN, PID_MOT_R.Output);
  analogWrite(MOT_L_PWM_PIN, PID_MOT_L.Output);

}  // End update_speeds_from_PID_controller

//////////////////////////////////////////////////////////////////
// FUNCION: void update_encoder_abs_value(action_t type_action) //
// In function of the sense turning of wheels the encoder could //
// increment or decrement the absolute counter value            //
//////////////////////////////////////////////////////////////////
void update_encoder_abs_value(action_t type_action)
{
  // Calculate the delta increment value from encoders right and left
  MOT_R.delta_encoder = (long) MOT_R.encoderDER - (long) MOT_R.last_value_encoder;
  MOT_L.delta_encoder = (long) MOT_L.encoderIZQ - (long) MOT_L.last_value_encoder;

  // Comprobate the action  
  switch(type_action)
  {
    case FORWARD:
      // Both absolute counter encoder increment the value
      MOT_R.encoderAbs += MOT_R.delta_encoder;
      MOT_L.encoderAbs += MOT_L.delta_encoder;
    break;

    case BACKWARD:
      // Both absolute counter encoder decrement the value
      MOT_R.encoderAbs -= MOT_R.delta_encoder;
      MOT_L.encoderAbs -= MOT_L.delta_encoder;    
    break;

    case TURN_LEFT:
      // Right encoder wheel increment the value but the left encoder wheel decrement the value
      MOT_R.encoderAbs += MOT_R.delta_encoder;
      MOT_L.encoderAbs -= MOT_L.delta_encoder; 
    break;

    case TURN_RIGHT:
      // Right encoder wheel decrement the value but the left encoder wheel increment the value
      MOT_R.encoderAbs -= MOT_R.delta_encoder;
      MOT_L.encoderAbs += MOT_L.delta_encoder;
    break;     
  }

  // Update the last value encoder with the actual counter of incremental encoder
  MOT_R.last_value_encoder = MOT_R.encoderDER;
  MOT_L.last_value_encoder = MOT_L.encoderIZQ;

}  // End update_encoder_abs_value

/////////////////////////////////////////////////
// FUNCION: Send data encoder (Value of count) //
/////////////////////////////////////////////////
void send_data_encoder(void) 
{
  // Define the local variables
  byte byte_auxiliar[4];
  unsigned long tAabs = micros();
  unsigned long tBabs = micros();
  
  byte_auxiliar[3] = (MOT_R.encoderAbs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (MOT_R.encoderAbs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (MOT_R.encoderAbs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (MOT_R.encoderAbs & 0x000000FF);

  Serial.println("N");              // Se manda una 'N'

  //Serial.print("N ");
  //Serial.print("  ");

  Serial.write(byte_auxiliar, 4);

  //DEBUG
  //Serial.print(" Dep EncoderIZDO: ");
  //Serial.print(encoderAAbs);
  //Serial.print("  ");
  //Serial.print(" Tiempo Izdo: ");
  //Serial.println(tAabs);
  // FIN DEBUG

  byte_auxiliar[3] = (tAabs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (tAabs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (tAabs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (tAabs & 0x000000FF);

  Serial.write(byte_auxiliar, 4);

  //DEBUG
  //Serial.print("  Dep tAABS: ");
  //Serial.print(tAabs);
  //Serial.print("  ");
  // FIN DEBUG

  byte_auxiliar[3] = (MOT_L.encoderAbs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (MOT_L.encoderAbs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (MOT_L.encoderAbs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (MOT_L.encoderAbs & 0x000000FF);

  Serial.write(byte_auxiliar, 4);

  //DEBUG
  //Serial.print(" EncoderDCHO: ");
  //Serial.print(encoderBAbs);
  //Serial.print("  ");
  //Serial.print(" Tiempo dcho: ");
  //Serial.println(tBabs);
  //Serial.print("  ");
  // FIN DEBUG

  byte_auxiliar[3] = (tBabs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (tBabs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (tBabs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (tBabs & 0x000000FF);

  Serial.write(byte_auxiliar, 4);
  //DEBUG
  //Serial.print("  Dep tBABS: ");
  //Serial.print(tBabs);
  // FIN DEBUG
  Serial.println("P");   ///// Se manda una 'P'

}  // End of send_data_encoder

///////////////////////////////////////////
// FUNCION:  Lola                        //
// Loop function                         //
///////////////////////////////////////////
void Lola(void)
{

  long t_start_tasks = 0;

  while (1)
  {
    // Capture the start time of cycle tasks
    t_start_tasks = millis();
    
    // Check if there is a byte in buffer reception from Serial Port
    if (Serial.available() > 0)
      {
        char order[0] = "Z";
        Serial.readBytes(order, 1);
        // Every time a command is going to be reveived the error is reset if it's not asking for an error code
        if (order[0] != "E")
          type_error = NO_ERROR;
        
        // Analyze the order received in the buffer reception from Serial Port
        // 'V': command velocity
        // '?': stop and reset
        // 'E': an error occur
        // 'Q': send the incremental data encoder
        // 'N': send the data encoder (abs count encoder)        
        analyze_order(order[0]);
      }
    
    // Each state performs a different movement
    switch (state)
    {
      case RESET:      
        // Just in case an error produced the movement of the motors
        stop_motors();
        // Reset the abs count value from encoders
        MOT_R.encoderAbs = 0;
        MOT_L.encoderAbs = 0;        
      break;

      case MOVE:
        // Update the speeds
        update_speeds_from_PID_controller(); 
      break;

      case STOP:
        // Just in case an error produced the movement of the motors
        stop_motors();
      break;

      default:
      break;
    } // End of switch (STATE)

    // Delay
    // This action avoid the effect of Jitter
    delay(PID_MOT_R.ts - (millis() - t_start_tasks));
  } // End of while(1)
} // End of loop
