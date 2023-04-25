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

#define IGNORE_PULSE   1400 // time in micros to ignore encoder pulses if faster
// Direction of movement
unsigned char dir_right,dir_left;
unsigned char velIZQ,velDER;
unsigned char flag_move_motor=0;
unsigned int veloc_right;
unsigned int veloc_left;

// Auxiliar variables to keep micros() at encoders
unsigned long tr,tl;
// Auxiliar variables to filter false impulses from encoders
volatile unsigned long auxr=0,auxl=0;
// Variables to keep each encoder pulse
volatile unsigned int encoderIZQ = 0, encoderDER = 0;
unsigned int aux_encoderIZQ = 0, aux_encoderDER = 0;

volatile signed int encoder = 0;
// Indicate the PWM that is applied to the motors
unsigned int SPEED_INI_L=255;  // 170
unsigned int SPEED_INI_R=255;  // 100
unsigned int SPEED_INI_L_LIM=255;  // 170
unsigned int SPEED_INI_R_LIM=255;  // 100
int velr = SPEED_INI_R;
int vell = SPEED_INI_L;
int error = 0;
int encoder_ant;
// FSM's STATES
unsigned char STATE = 0;
#define RESET_STATE          0
#define MOVE_DIF_SPEED     10

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
#define PASOS_POR_VUELTA 712.6
//Fijamos Kdato para velocidad de 3,5 rad/s a 127 (maxima
#define Kdato 36.28

unsigned long atr = 0;
int encoderIZQ_aux_can = 0;
int encoderDER_aux_can = 0;
long encoderAAbs = 0, tAabs;
long encoderBAbs = 0, tBabs;

//////////////////////////////////
//      ERROR VARIABLE          //
// IDENTIFICATION ERRORS CODES  //
//////////////////////////////////
unsigned char ERROR_CODE = 0;
#define NO_ERROR              0
#define NO_NUMBER             1   // Waiting for a number, time-out
#define OUT_RANGE             2   // Received number out of range
#define SPEED_OUT_RANGE       3   // Received speed out of range
#define RR_OUT_RANGE          4   // Radio Ratio out of range
#define NO_AVAILABLE          5   // Received Command non available
#define INERTIA_LIMIT_ERROR   6   // Distance lower than inertia limit

char order[1];

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCION: SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{ 
  // Add the interrupt lines for encoders
//  attachInterrupt(digitalPinToInterrupt(MOT_R_ENC_B_PIN), cuentaDER, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(MOT_L_ENC_B_PIN), cuentaIZQ, CHANGE);
  attachInterrupt(0, cuentaDER, CHANGE);
  attachInterrupt(1, cuentaIZQ, CHANGE);

  //Battery pin for voltaje measurement
  pinMode(BAT_PIN,         INPUT);

  //Dip switch for configuration
  pinMode(SW1_PIN,  INPUT_PULLUP);
  pinMode(SW2_PIN,  INPUT_PULLUP);
  pinMode(SW3_PIN,  INPUT_PULLUP);
  pinMode(SW4_PIN,  INPUT_PULLUP);
  pinMode(SW5_PIN,  INPUT_PULLUP);
  pinMode(SW6_PIN,  INPUT_PULLUP);
  pinMode(SW7_PIN,  INPUT_PULLUP);
  pinMode(SW8_PIN,  INPUT_PULLUP);

  //Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // set all the motor control pins to outputs
  pinMode(MOT_R_PWM_PIN, OUTPUT);
  pinMode(MOT_L_PWM_PIN, OUTPUT);
  
  pinMode(MOT_R_A_PIN, OUTPUT);
  pinMode(MOT_R_B_PIN, OUTPUT);
  pinMode(MOT_L_A_PIN, OUTPUT);
  pinMode(MOT_L_B_PIN, OUTPUT);

  // set encoder pins to inputs
  pinMode(MOT_L_ENC_B_PIN, INPUT);
  pinMode(MOT_R_ENC_B_PIN, INPUT);

  //L RGB LED
  pinMode(L_RED_PIN,      OUTPUT);
  pinMode(L_GRE_PIN,      OUTPUT);
  pinMode(L_BLU_PIN,      OUTPUT);
  
  //R RGB LED
  pinMode(R_RED_PIN,      OUTPUT);
  pinMode(R_GRE_PIN,      OUTPUT);
  pinMode(R_BLU_PIN,      OUTPUT);

  //F Ultrasound sensor
  pinMode(F_US_TRIG,      OUTPUT);
  pinMode(F_US_ECHO,      INPUT);
  //L Ultrasound sensor
  pinMode(L_US_TRIG,      OUTPUT);
  pinMode(L_US_ECHO,      INPUT);
  //R Ultrasound sensor           
  pinMode(R_US_TRIG,      OUTPUT);
//  pinMode(R_US_ECHO,      INPUT); Pin 53 (coincide con el pin CAN necesario en Mega)
  //B Ultrasound sensor
  pinMode(B_US_TRIG,      OUTPUT);
  pinMode(B_US_ECHO,      INPUT);
  
  // set buttons pins
  pinMode(PIN_FORWARD, INPUT_PULLUP);
  pinMode(PIN_BACKWARD, INPUT_PULLUP);
  pinMode(PIN_LEFT, INPUT_PULLUP);
  pinMode(PIN_RIGHT, INPUT_PULLUP);
  
  pinMode(LED, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  digitalWrite(MOT_R_A_PIN, LOW);
  digitalWrite(MOT_R_B_PIN, LOW);    
  digitalWrite(MOT_L_A_PIN, LOW);
  digitalWrite(MOT_L_B_PIN, LOW);

  analogWrite(MOT_R_PWM_PIN, 0);
  analogWrite(MOT_L_PWM_PIN, 0);

  Serial.begin(115200);      //init the serial port
  
  Serial.print("LOLA INI ");
}  // End of setup()


//////////////////////////////////////////////////
// FUNCION: cuentaDER()
//  Right encoder interrupt using only one pin
//////////////////////////////////////////////////
void cuentaDER()
{
  tr=micros();
  // if pulse is too fast from previoius is ignored
  if (tr-auxr>(unsigned long)IGNORE_PULSE)
  {    
    auxr=tr;
    encoderDER++;    //Add one pulse
  }  
}  // end of cuentaDER

///////////////////////////////////////////////////
// FUNCION: cuentaIZQ()
//  Left encoder interrupt using only one pin
//////////////////////////////////////////////////
void cuentaIZQ()
{
  tl=micros();
  // if pulse is too fast from previoius is ignored
  if (tl-auxl>(unsigned long)IGNORE_PULSE)
  {
    auxl=tl;
    encoderIZQ++;  //Add one pulse
  }
}  // end of cuentaIZQ


///////////////////////////////////////////
// FUNCION: move_motors()                //
//              MOVE MOTORS              //
//  dir_right (1: foward / 0: backwards) //
//  dir_left  (1: foward / 0: backwards) //
///////////////////////////////////////////
void move_motors() 
{
  // now turn off motors
  // Adaptation for L298n
  unsigned int inhib_r = 0xBB, inhib_l = 0xEE;

  // When starting reset all the encoders
  //encoderIZQ =  aux_encoderIZQ ;
  //encoderDER =  aux_encoderDER;   
  encoder = encoder_ant = 0;

  //Deactivate both motor's H-bridge
  PORTA &= 0xAA;

  velr = SPEED_INI_R;
  vell = SPEED_INI_L;

  if (!velr)
    PORTB |= 0x08;            
  else {
    inhib_r |= 0x44;
    analogWrite(MOT_R_PWM_PIN, velr);
  }

  if (!vell)
    PORTB |= 0x10;             

  else {
    inhib_l |= 0x11;
    analogWrite(MOT_L_PWM_PIN, vell);
  }  

  if (dir_right && dir_left)
    PORTA |= 0x05 & inhib_r & inhib_l;

  else if (!dir_right && dir_left)
    PORTA |= 0x41 & inhib_r & inhib_l;

  else if (dir_right && !dir_left)
    PORTA |= 0x14 & inhib_r & inhib_l;
  
  else
    PORTA |= 0x50 & inhib_r & inhib_l;
    
}  // End of move_motors()

///////////////////
// FUNCION: stop_motors()
//  STOP_MOTORS  //
///////////////////
void stop_motors() 
{
  PORTB |= 0x3 << 3;                              
  PORTA &= 0xAA;
  //We will fix the same duty cycle for the PWM outputs to make the braking spped equal!
  analogWrite(MOT_R_PWM_PIN, 255);
  analogWrite(MOT_L_PWM_PIN, 255);
}  // End of stop_motors()

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCION: speed_normalization()
//
//  Speeds are normalized in order to work 
//  at maximum speed give as SPEED_INI_X
//
//////////////////////////////////////////////////
void speed_normalization()
{
  if (velr>SPEED_INI_R)
    {
      vell -= (velr-SPEED_INI_R);
      velr = SPEED_INI_R;
      if (vell<0)
        vell=0;          
    }
  if (vell>SPEED_INI_L)
    {
      velr -= (vell-SPEED_INI_L);
      vell = SPEED_INI_L;
      if (velr<0)
        velr = 0;
    }
} // end of speed_normalization

///////////////////////////
// FUNCION: void loop() 
void loop() 
{
  // put your main code here, to run repeatedly:
  STATE=RESET_STATE;
  Lola();
}  // end of loop()

///////////////////////////////////////////
// FUNCION:  READ_NUMBER                 //
//  Read a number from the serial port with <number> digits  //
///////////////////////////////////////////////////////////////
short int read_number(int n_digits) {
  char speed[8];

  // Wait to be sure the bytes have arrived
  delay(5);
  //  sprintf(speed,"%d",SERIA.available());
  //  SERIA.println(speed);
  if (digitalRead(SW5_PIN) == LOW) {
    if (Serial2.available() > n_digits - 1)
    {
      Serial2.readBytes(speed, n_digits);
      speed[n_digits] = '\0';  // Append a NULL character to terminate the string! Not needed if initialized with char speed[5] = {0}...
      //      SERIA.println(speed);

      return (unsigned int) atoi(speed);
    }
    else
    {
      ERROR_CODE = NO_NUMBER;
      return 0;
    }
  }
  else {
    if (Serial.available() > n_digits - 1)
    {
      Serial.readBytes(speed, n_digits);
      speed[n_digits] = '\0';  // Append a NULL character to terminate the string! Not needed if initialized with char speed[5] = {0}...
      //      SERIA.println(speed);

      return (unsigned int) atoi(speed);
    }
    else
    {
      ERROR_CODE = NO_NUMBER;
      return 0;
    }
  }
}  // End of read_number()

///////////////////////////////////////////
// FUNCION:   ANALYZE_ORDER              //
// Parse command information and trigger //
// all the necessary functions           //
///////////////////////////////////////////
void analyze_order() {
  char str[35];
  float s, sl, sr;
  int num, n;

  switch (order[0])
  {
    case   'V':
      velDER = read_number(3);
      velIZQ = read_number(3);

      if (velDER == 255)
        velDER = 254;
      if (velIZQ == 255)
        velIZQ = 254;

      if (STATE == RESET_STATE)
      {
        flag_move_motor = 1;
        SPEED_INI_R = SPEED_INI_L = 128;
      }

      if (velDER == 0  && velIZQ == 0 )
      {
        STATE = RESET_STATE;
        stop_motors();
        return;
      }
      else
        STATE = MOVE_DIF_SPEED;
      movimientos_vel();
      break;

    // Stop motors and calcucompute the new positions
    case 0x3F: // '?'
      stop_motors();
      STATE = RESET_STATE;
      break;

    // The error is sent back by adding 0x30 to the error code
    case 0x45: // 'E'
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.write(0x45);  // 'E'
        Serial2.write(0x30 + ERROR_CODE);
        Serial2.println("");
      }
      else {
        Serial.write(0x45);  // 'E'
        Serial.write(0x30 + ERROR_CODE);
        Serial.println("");
      }
      break;
    case 'Q': // 'Q'
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("\nQ:");
        Serial2.print(encoderIZQ);
        Serial2.print(" ");
        Serial2.println(encoderDER);
      }
      else {
        Serial.print("\nQ:");
        Serial.print(encoderIZQ);
        Serial.print(" ");
        Serial.println(encoderDER);
      }
      break;

    case 'N': //'N'    Datos de los encoders en valor absoluto
      enviar_datos_encoder();
      break;

    default:
      ERROR_CODE = NO_AVAILABLE;
  }
}  // End of analyze_order()

///////////////////////////////////////////
// FUNCION: void update_speeds(unsigned char reset)
// This function perform a PID for speeds control:
// The global variables with the objective speeds are:
//          veloc_right
//          veloc_left
//
// The global variables with the speed limits are:
//          SPEED_INI_R
//          SPEED_INI_L

// Input parameter:
//  unsigned char reset, if reset==1 the initial values are set in variables.
////////////////////////////////////////////////////////////////////////
void update_speeds(unsigned char reset)
{
  static unsigned int encI_aux = 0, encD_aux = 0;
  unsigned int temp_encDER, temp_encIZQ;
  float aux_floatDER, aux_floatIZQ;
  unsigned long t;
  long tiempo;

  float errorD, errorI;
  static float ant_errorD = 0, ant_errorI = 0;

  unsigned int real_vI, real_vD;

  static float KP = (float)0.3; //0.7
  static float KD = (float)0.5; // 1

  if (reset == 1)
  {
    encI_aux = encoderIZQ;
    encD_aux = encoderDER;
    ant_errorD = 0;
    ant_errorI = 0;
    atr = micros();
    return;
  }

  tr = micros();
  tiempo = (long)tr - (long)atr;
  if (tiempo < 90000)
    return;
  atr = tr;

  actualizar_encoder_abs();

  temp_encDER = encoderDER;
  temp_encIZQ = encoderIZQ;

  real_vD = (unsigned int) ((float)temp_encDER - (float)encD_aux) / ((float)tiempo / (float)1000000);
  real_vI = (unsigned int) ((float)temp_encIZQ - (float)encI_aux) / ((float)tiempo / (float)1000000);

  encD_aux = temp_encDER;
  encI_aux = temp_encIZQ;
  
  aux_floatDER = (float)veloc_right - (float)real_vD;
  aux_floatIZQ = (float)veloc_left  - (float)real_vI;

  errorD = ant_errorD - aux_floatDER;
  errorI = ant_errorI - aux_floatIZQ;
  ant_errorD = aux_floatDER;
  ant_errorI = aux_floatIZQ;

  // Implement PID (just PD)
  aux_floatDER = (float) aux_floatDER * KP - (float) errorD * KD;
  aux_floatIZQ = (float) aux_floatIZQ * KP - (float) errorI * KD;

  // Redondeos al entero ms cercano
  if (aux_floatDER > 0)     aux_floatDER += 0.5;
  if (aux_floatDER < 0)     aux_floatDER -= 0.5;
  if (aux_floatIZQ > 0)     aux_floatIZQ += 0.5;
  if (aux_floatIZQ < 0)     aux_floatIZQ -= 0.5;

  if ((velr + aux_floatDER) < 0)
    velr = 0;
  else
    velr += (int)aux_floatDER;
  if ((vell + aux_floatIZQ) < 0)
    vell = 0;
  else
    vell += (int)aux_floatIZQ;

  if (velr < 0)
    velr = 0;
  if (vell < 0)
    vell = 0;
  if (velr > SPEED_INI_R)
    velr = SPEED_INI_R;
  if (vell > SPEED_INI_L)
    vell = SPEED_INI_L;

    if (vell > SPEED_INI_L || velr > SPEED_INI_R)
    {
      speed_normalization();
    }

  // Write, as PWM duty cycles, the speeds for each wheel
  analogWrite(MOT_R_PWM_PIN, velr);
  analogWrite(MOT_L_PWM_PIN, vell);

}  // fin de void update_speeds()

///////////////////////////////////////////
// FUNCION: void actualizar_encoder_abs()
void actualizar_encoder_abs()
{
  if (dir_left == 0)
  {
    /*
    encoderIZQ_aux_can = (long)encoderIZQ_aux_can - (long)encoderIZQ;
    encoderAAbs = (long)encoderAAbs + (long)encoderIZQ_aux_can;
    encoderIZQ_aux_can = encoderIZQ;
    */

    encoderIZQ_aux_can = (long)encoderIZQ - (long)encoderIZQ_aux_can;
    encoderAAbs = (long)encoderAAbs - (long)encoderIZQ_aux_can;
    encoderIZQ_aux_can = (long)encoderIZQ;

  }
  else
  {
    /* 
    encoderIZQ_aux_can = (long)encoderIZQ_aux_can - (long)encoderIZQ;
    encoderAAbs = (long)encoderAAbs - (long)encoderIZQ_aux_can;
    encoderIZQ_aux_can = encoderIZQ;
    */
    encoderIZQ_aux_can = (long)encoderIZQ - (long)encoderIZQ_aux_can;
    encoderAAbs = (long)encoderAAbs + (long)encoderIZQ_aux_can;
    encoderIZQ_aux_can = (long)encoderIZQ;
  }
  if (dir_right == 0)
  {
    /*
    encoderDER_aux_can = (long)encoderDER_aux_can - (long)encoderDER;
    encoderBAbs = (long)encoderBAbs + (long)encoderDER_aux_can;
    encoderDER_aux_can = encoderDER;
    */
    encoderDER_aux_can = (long)encoderDER - (long)encoderDER_aux_can;
    encoderBAbs = (long)encoderBAbs - (long)encoderDER_aux_can;
    encoderDER_aux_can = encoderDER;
  }
  else
  {
    /*
    encoderDER_aux_can = (long)encoderDER_aux_can - (long)encoderDER;
    encoderBAbs = (long)encoderBAbs - (long)encoderDER_aux_can;
    encoderDER_aux_can = encoderDER;
    */

    encoderDER_aux_can = (long)encoderDER - (long)encoderDER_aux_can;
    encoderBAbs = (long)encoderBAbs + (long)encoderDER_aux_can;
    encoderDER_aux_can = encoderDER;
  }

  /*
  // DEBUG PULSOS ENCODERS
  Serial.print("EncoderIZDO:");
  Serial.print(encoderAAbs);
  Serial.print(", ");
  Serial.print("EncoderDCHO: ");
  Serial.println(encoderBAbs);
  */

}  // fin actualizar_encoder_abs

///////////////////////////////////////////
// FUNCION: void movimientos_vel()
void movimientos_vel()
{
  actualizar_encoder_abs();

  if (velDER == 0)
  {
    if (veloc_right != 0)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    veloc_right = 0;
    dir_right = 0;
  }
  else if (velDER < 128)
  {
    if (dir_right == 0)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    //veloc_right=10+280*(unsigned int)velDER/128;
    veloc_right=(unsigned int)(PASOS_POR_VUELTA*velDER/(2*PI*Kdato)+0.5);

    dir_right = 1;
  }
  else
  {
    if (dir_right == 1)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    //velr=255+127-velDER;
    velDER = 256 - velDER;
    //veloc_right=10+280*(unsigned int)velDER/128;
    veloc_right=(unsigned int)(PASOS_POR_VUELTA*velDER/(2*PI*Kdato)+0.5);

    dir_right = 0;
  }

  if (velIZQ == 0)
  {
    if (veloc_left != 0)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    veloc_left = 0;
    dir_left = 0;
  }
  else if (velIZQ < 128)
  {
    if (dir_left == 0)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    //veloc_left=10+280*(unsigned int)velIZQ/128;
    veloc_left=(unsigned int)(PASOS_POR_VUELTA*velIZQ/(2*PI*Kdato)+0.5);
    dir_left = 1;
  }
  else
  {
    if (dir_left == 1)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    //vell=255+127-velIZQ;
    velIZQ = 256 - velIZQ;
    //veloc_left=10+280*(unsigned int)velIZQ/128;
    veloc_left=(unsigned int)(PASOS_POR_VUELTA*velIZQ/(2*PI*Kdato)+0.5);
    dir_left = 0;
  }
  
  if (veloc_left == 0)
    SPEED_INI_L = 0;
  //  else
  //    SPEED_INI_L=255;
  if (veloc_right == 0)
    SPEED_INI_R = 0;
  //  else
  //    SPEED_INI_R=255;

  if (flag_move_motor == 1)
  {
    move_motors();
    //    Serial.print(" Reset movimiento ");
    flag_move_motor = 0;
  }
  update_speeds(1);

} // fin de movimientos_vel()

///////////////////////////////////////////
// FUNCION: Enviar datos encoder
/////////////////////////////////////////////////////////////////////
void enviar_datos_encoder() 
{
  byte byte_auxiliar[4];

  actualizar_encoder_abs();
  tAabs = micros();
  tBabs = micros();
  
  
  byte_auxiliar[3] = (encoderAAbs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (encoderAAbs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (encoderAAbs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (encoderAAbs & 0x000000FF);

  Serial.println("N");   ///// Se manda una 'N'

  //Serial.print("N ");
  //Serial.print("  ");

  Serial.write(byte_auxiliar, 4);
  
  
  /*
  //DEBUG
  Serial.print(" Dep EncoderIZDO: ");
  Serial.print(encoderAAbs);
  Serial.print("  ");
  Serial.print(" Tiempo Izdo: ");
  Serial.println(tAabs);
  // FIN DEBUG
  */

  
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

  byte_auxiliar[3] = (encoderBAbs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (encoderBAbs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (encoderBAbs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (encoderBAbs & 0x000000FF);

  Serial.write(byte_auxiliar, 4);


/*
//DEBUG
Serial.print(" EncoderDCHO: ");
Serial.print(encoderBAbs);
Serial.print("  ");
Serial.print(" Tiempo dcho: ");
Serial.println(tBabs);
Serial.print("  ");
// FIN DEBUG
*/


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

}  //// Fin enviar los datos de los encoders

///////////////////////////////////////////
// FUNCION:  Lola                        //
// Loop function                         //
///////////////////////////////////////////
void Lola()
{
  unsigned int auxiliar_uns_int;
  while (1)
  {
    if (Serial.available() > 0)
      {
        order[0] = 'Z';
        Serial.readBytes(order, 1);
        // Every time a command is going to be reveived the error is reset if it's not asking for an error code
        if (order[0] != 'E')
          ERROR_CODE = NO_ERROR;
        analyze_order();
      }
    
    // Each state performs a different movement
    switch (STATE)
    {
      case   RESET_STATE:      
        // Just in case an error produced the movement of the motors
        stop_motors();
        encoderIZQ = 0, encoderDER = 0;
        encoderIZQ_aux_can = 0, encoderDER_aux_can = 0;
        encoderAAbs = 0, encoderBAbs = 0;
        break;
      case MOVE_DIF_SPEED:
        auxiliar_uns_int=(int)((float)SPEED_INI_L*1.1);
        if (auxiliar_uns_int>SPEED_INI_L_LIM)
          SPEED_INI_L=SPEED_INI_L_LIM;
        else
          SPEED_INI_L=auxiliar_uns_int;
        SPEED_INI_R=SPEED_INI_L;
            
        // PID for speeds update
        update_speeds(0);
        break;
      default:
        break;
    } // End of switch (STATE)
  } // End of while(1)
} // End of loop
