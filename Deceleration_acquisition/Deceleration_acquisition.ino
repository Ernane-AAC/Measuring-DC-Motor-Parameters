// UNVERSIDADE FEDERAL DE UBERLÂNDIA - Faculdade de Engenharia Elétrica
// Subject: DSP-EPS - Prof. Ernane A. A. Coelho - http://lattes.cnpq.br/9183492978798433
// CODE for motor deceleration data acquisition
//  * run on Arduino Lab System (Arduino UNO - Atmega328p)
//  * Sample rate: 100Hz 
//  * The code store the encoder pulse counting during the deceleration process
//  * The buffer size for counting depends on the sample rate and motor mechanical time constant
//  * The motor is driven by the L298N Bridge controlled by the PWM outputs of Timer1
//  * Timer 1 is configure to provide a triangular PWM carrier at 5kHz
//  * PWM outputs are programmed to generate the unipolar modulation. This will imply 
//      a PWM frequency at 10kHz seen by the motor, reduncing the armature current ripple
//  * The sample rate is generated as a submultiple of Timer1's interrupt request rate       
//  * HMI:    -> 16x2 I2C LCD
//            -> 4x4 I2C Keypad


#include <Keypad_I2C.h>        // Keypad libs
#include <Keypad.h>
#include <Wire.h>
#define I2CADDR 0x21           // Keypad i2c expansor address (PCF8574)
#include <LiquidCrystal_I2C.h> // LCD I2C Library
#define LCD_col 16             // number of LCD columns
#define LCD_lin  4             // number of LCD lines
#define LDCi2c_Address  0x27   // LCD I2C address
#define BUFSIZE  100           // buffer size to store the deceleration curve - adjust the size as needed             

// ============ kepad  config ====================================
const byte ROWS = 4;           // number of keypad lines
const byte COLS = 4;           // number of keypad columns
char keys[ROWS][COLS] = {      // key specification
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
// keyboard rows and columns vectors specify the keyboard scan lines and return 
//  lines according to the connection to the PCF8574 I/O port bits
byte rowPins[ROWS] = {4, 5, 6, 7}; //connection to the row pinouts of the keypad
byte colPins[COLS] = {0, 1, 2, 3}; //connection to the column pinouts of the keypad

TwoWire *jwire = &Wire;       //test passing pointer to keypad lib
Keypad_I2C kpd( makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR, PCF8574, jwire );

// ============ LCD config =======================================
LiquidCrystal_I2C lcd(LDCi2c_Address,LCD_col,LCD_lin);

unsigned int delta=40;            // PWM duty cycle increment or decrement step -> 2.5%  of the TOP
unsigned int Comp_A=800;          // Compare A value -> 50% (TOP) 1600*0.5=800
unsigned int Comp_B=800;          // Compare B value -> 50% (TOP) 1600*0.5=800
uint16_t int01_counter;           // INT0 and INT1 counter
uint16_t encoder_pulses[BUFSIZE]; // buffer to store the deceleration curve - adjust the size as needed
uint16_t index;                   // index for encoder_pulses vector
uint32_t T_pr;                    // previous time - used with millis() to synchronize the keypad sample rate
uint8_t timer1_int_counter;       // timer1 interrupt counter 1 - encoder sample synchronization
uint8_t curve;                    // indicates the existence of a deceleration curve for the serial transmission
uint16_t timer1_int_counter2;     // onboard LED synchronization - 1Hz (operation signaling)
float vel_rps;                    // velocity in revolutions per second
char direction;                   // direction of rotation
float count_to_rps;               // constant to convert encoder pulse counting in rps
unsigned char LCD_counter;        // synchronizes LCD update every second

//=========== SETUP ==============================================
void setup() 
{
 // I/O Configuration
 //  PB0(pin  8 Arduino) -> GPIO output   -> Enable gates bridge A
 //  PB1(pin  9 Arduino) -> Timer1 output -> OC1A (Timer/Counter1 output compare match A output)
 //  PB2(pin 10 Arduino) -> Timer1 output -> OC1B (Timer/Counter1 output compare match B output)
 //  PB5(pino 13 Arduino)-> GPIO output   -> onboard LED
 PORTB=0x00;  // reset  PB bits
 DDRB = 0x27; // DDRB= 0010 0111 -> Bits PB0, PB1, PB2 e PB5 to output, other pins as input

  Serial.begin(115200); //starts serial interface
 
  jwire->begin( );    // arrow operator - as in keypad lib example
  kpd.begin( );       // starts keypad
  lcd.begin();        // starts LCD
  lcd.backlight();    // turn on the LCD backlight
  print_home();       // print the main menu on the LCD
  Config_Timer1();    // set the Timer1 configuration
  Config_Int0_Int1(); //set external interrupts configuration
  // ====== initializations =============
  T_pr=millis(); 
  LCD_counter=0;
  timer1_int_counter=0;
  int01_counter;
  index=400;
  curve=0;
  direction=1;
  count_to_rps=1.0/4/11/45*100; // rps constant conversion /(4 ints per encoder pulse)/(encoder pulses per turn)/(gerbox ratio)*sample frequency
}

//=========== LOOP ==============================================
void loop() 
{
  char key=0;
  uint16_t k;
  
  if(millis()-T_pr>250) //sample rate at 4Hz
  {
   T_pr=millis();
   LCD_counter++; 
   key = kpd.getKey(); 
   if (key !=0)
      { 
       if (key=='1')
          {
            PORTB |= 0x01; //enable bridge A gates 
          }   
       if (key=='0')
          {
            PORTB &= 0xFE; //disable bridge A gates
            Comp_A=800;    // restart compare values to 50%
            Comp_B=800;
            OCR1A=Comp_A; // load compare A  register   
            OCR1B=Comp_B; // load compare B  register  
          }                 
       if (key=='A')
          {
            Comp_A=Comp_A + delta;        // increases the duty cycle
            if(Comp_A > 1520)Comp_A=1520; // duty cycle is clamped to 95%
            Comp_B=1600 - Comp_A;         // calculates the complementary duty cycle
            OCR1A=Comp_A;                 // load compare A  register       
            OCR1B=Comp_B;                 // load compare B  register  
          }
       if (key=='B')
          {
            Comp_A=Comp_A - delta;        // decreases the duty cycle
            if(Comp_A < 80)Comp_A=80;     // minumum duty cycle  is clamped to 5%
            Comp_B=1600 - Comp_A;         // calculates the complementary duty cycle
            OCR1A=Comp_A;                 // load compare A  register       
            OCR1B=Comp_B;                 // load compare B  register 
          } 
       if (key=='#')
          {
            PORTB &= 0xFE; // disable bridge A gates
            index=0;       // reset index to save deceletion curve in the buffer
            curve=1;       // there wiil be a curve to transmit
            Comp_A=800;    // restart compare values to 50%
            Comp_B=800;
            OCR1A=Comp_A;  // load compare A  register   
            OCR1B=Comp_B;  // load compare B  register  
          }   
      }
  }
  if (LCD_counter==4)
  {
    LCD_counter=0;
    lcd.setCursor(5,1); 
    lcd.print(vel_rps); //print speed on the LCD
    lcd.print("   ");   //display manipulation, negative values use distinct positions on display
  }
  if((curve==1) && (index>=BUFSIZE)) // transmit deceleration curve if it exists
  {                              // to simplify the transmission only the integer counting is transmitted
    curve=0;                     // this curve was transmitted
    Serial.println("New_curve");
    for(k=0;k<100;k++)
       {
        Serial.println(encoder_pulses[k]);
       }
    Serial.println("End_curve");
  }
}

// ========== External Interrupts Services ========================
ISR(INT0_vect)       // install INT0 service
{                    // INT0-> PD2 (pin 2 of the Arduino connector)
  int01_counter++;
  if(PIND & 0x04)   //check encoder channel A and B phase
     {
      if (PIND & 0x08) 
        {
          direction = 1;
        }
       else
        {
          direction =-1;
        } 
     }  
}
ISR(INT1_vect)       // install INT1 service
{                    // INT1-> PD3 (pin 3 of the Arduino connector)
  int01_counter++;
}

// ========== Timer 1 ISR ========================================
ISR(TIMER1_OVF_vect){
  timer1_int_counter++;
  timer1_int_counter2++;
  if (timer1_int_counter>=50)   //5kHz/50 -> 100Hz sample rate
    {
      timer1_int_counter=0;
      vel_rps=(float)int01_counter*count_to_rps*direction; // convert counting to rps
      if(index < BUFSIZE)     
         {
          encoder_pulses[index]=int01_counter; //save encoder counting data
          index++;
         }
      int01_counter=0;             // starts new counting
    }
    if(timer1_int_counter2>=2500)  // blink onboard LED at 1Hz
    {
      timer1_int_counter2=0;
      PINB |=0x20;                 // toggle onboard LED bit
    } 
}

// =========== External Interrupsts configuration ================
void   Config_Int0_Int1(void)
{
    // ===== EICRA – External Interrupt Control Register A =====
   //ISCx1 ISCx0 Description (x=0 ->INT0; x=1 ->INT1)
   //  0     0   The low level of INTx generates an interrupt request.
   //  0     1   Any logical change on INTx generates an interrupt request.
   //  1     0   The falling edge of INTx generates an interrupt request.
   //  1     1   The rising edge of INTx generates an interrupt request.
   //  Bit  7 6 5 4   3     2     1    0
   //(0x69) – – – – ISC11 ISC10 ISC01 ISC00 
   //       0 0 0 0   0     1     0     1  
 EICRA = 0x05; // Config INT0 and INT to acknowledge interrupt at any logical change
   // ===== EIMSK – External Interrupt Mask Register =====
   // INTx -> 0-disable INTx; 1-enable INTx
   //       Bit  7 6 5 4 3 2   1    0
   //0x1D (0x3D) – – – – – – INT1 INT0 
   //           0 0 0 0 0 0   1    1  
 EIMSK = 0x03;//enable INT0 and INT1 
}

// ============= Timer1 configuration ============================
void Config_Timer1(void)     // config timer 1 to provide triangular PWM carrier at 5kHz
{            
//--------------------------------------------------------------- 
//TCCR1A - Timer/Counter1 Control Register A
//Bit        7      6      5      4     3     2     1      0
//(0x80) COM1A1 COM1A0 COM1B1 COM1B0    –     –  WGM11  WGM10  -> Waveform generation Mode=10 -> WGM11:WGM10=10
//           1      0      1      0     0     0     1      0   -> Clear OC1A/OC1B on compare match when up-counting, 
//------------------------------------------------------------ -> Set OC1A/OC1B on compare match when down counting.
 TCCR1A=0xA2; //configura registro de controle A
//---------------------------------------------------------------           
//TCCR1B – Timer/Counter1 Control Register B
//Bit       7     6     5     4     3     2     1     0   -> Mode 10: PWM, phase correct, 10-bit -> TOP=1600
//(0x81) ICNC1 ICES1    –  WGM13 WGM12  CS12  CS11  CS10  -> Waveform generation Mode=3 -> WGM13:WGM12=10
//          0     0     0     1     0     0     0     1   -> Clk_Timer=Clk_CPU/1= 16MHz
//---------------------------------------------------------> Triangular PWM carrier => 16MHz/(2xTOP)= 16MHz/3200= 5000Hz           
 TCCR1B=0x11; //config control register A
//Compare registers:
//-> they have different meaning accoding the mode 
//-> always define after mode configuration
 OCR1A=Comp_A;// inicia compare A     
 OCR1B=Comp_B;// inicia compare B
//--------------------------------------------------------
//TIMSK1 – Timer/Counter1 Interrupt Mask Register
//Bit        7    6    5    4    3      2      1     0
//(0x6F)     –    – ICIE1   –    –  OCIE1B OCIE1A  TOIE1  -> timer 1 interrupt control
//           0    0    0    0    0      0      0     1    -> enable timer 1 overflow interrupt
//---------------------------------------------------------------  
 TIMSK1=0x01;   // enable timer 1 overflow interrupt
 ICR1=1600;     // config TOP to 1600 (Mod6 10)
 TCNT1=0;       // reset timer 1 counter
}   

// ================= Main MENU  ===============
void print_home(void){
  lcd.clear();
  lcd.setCursor(0,0);            // moves the cursor at the beginning of the first line
  //         0123456789012345 
  lcd.print("M Control   1:ON");
  lcd.setCursor(0,1);            // moves the cursor at the beginning of the second line
  lcd.print("RPS:            ");
  lcd.setCursor(16,0);           // moves the cursor at the beginning of the third line
  lcd.print("Speedup: A:R B:L"); // Speed up A-> Right   |    B -> Left      
  lcd.setCursor(16,1);           // moves the cursor to the beginning of the fourth line
  lcd.print("0:OFF   #:Dcurve"); // 0-> Turns switches off and stops motor | #  save and transmit the deceleration curve
}
