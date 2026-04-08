//The code belongs to the following tutorial and PCB project
//Tutorial: https://www.curiousscientist.tech/blog/stepper-motor-developing-platform
//PCB: https://www.pcbway.com/project/shareproject/Stepper_motor_developing_platform_d2e5fcf7.html
//Please consider supporting my work!

//GMG12864-06D display
#include <Arduino.h>
#include <U8g2lib.h> //Display fonts
#include <SPI.h>    //SPI Library
#include <Wire.h> //This is for i2C

#define TFT_CS PB12    //Chip select pin for the TFF (SPI2 default)
#define TFT_RST PA8    //Reset pin for the TFT
#define TFT_DC PA10    //A0 pin for the TFT
#define TFT_MOSI PB15  //MOSI of SPI2
#define TFT_SCLK PB13  //SCLK of SPI2
//SPI_2 - LCD
//CS    - PB12
//MISO  - Not applicable (We don't read from the display), Otherwise, it is PB14
//MOSI  - PB15
//SCK   - PB13

static SPIClass SPI_2(PB15, PB14, PB13); //SPI2 with the corresponding pins

U8G2_ST7565_ERC12864_ALT_F_4W_SW_SPI u8g2(U8G2_R0, TFT_SCLK, TFT_MOSI, /* cs=*/ TFT_CS, /* dc=*/ TFT_DC, /* reset=*/ TFT_RST);

bool fontColor = true; //1 = black, 0 = white

//--------------------------------
//Buttons and rotary encoder controls
#define DownButton PB8
#define UpButton PB9
#define RotarySW PB3
#define RotaryDT PB4
#define RotaryCLK PB5
//Note: 12 PPR EC11 encoder is preferred

//Statuses of the DT and CLK pins on the encoder
int CLKNow;
int CLKPrevious;
int DTNow;
int DTPrevious;
int RotaryButtonValue; //stores the button reading
unsigned long RotaryButtonTime = 0; //timing/debounce
unsigned long PushButtonTime = 0;
volatile long rotaryCounter = 0; //Rotary encoder click counting

//------------------------------------------------------------------------------------
//---TMC2209---
//------------------------------------------------------------------------------------
HardwareSerial Serial2 (PA3, PA2); //Defining UART2 as the secondary serial port for the motor
#include <TMCStepper.h> //Stepper motor driver library for TMCxxxx motors (TMC2209)
#define EN_PIN           PA6 // Enable
#define DIR_PIN          PA0 // Direction
#define STEP_PIN         PA1 // Step
#define SW_SCK           PA7 // Software Slave Clock (SCK)
#define SW_RX            PA3 // Serial receive pin
#define SW_TX            PA2 // Serial transmit pin
#define SERIAL_PORT Serial2 // PA2: TX, PA3: RX.
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2 (both zero (GND))
#define R_SENSE 0.11f //Sense resistor "R110" - There are 2 of them because of the 2 stepper coils.
#define MS1 PA5 //MS1
#define MS2 PA4 //MS2 - Microstepping 8, 16, 32, 64. Alternatively, address pins if UART used

bool shaft = false; //Shaft rotation direction

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS); //creating an instance called "driver"

#include <AccelStepper.h> //Accelstepper library for driving by the STEP&DIR pins
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);
bool buttonControl = false; //Bool variable to lock out mutually interfering motor driving routines
bool doIntro = false; //Bool variable to quickly enable/disable intro stepping in the setup
//------------------------------------------------------------------------------------
//---AS5600 magnetic position encoder---
//------------------------------------------------------------------------------------
int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0; //number of turns
float correctedAngle = 0; //tared angle - based on the startup value
float startAngle = 0; //starting angle
float totalAngle = 0; //total absolute angular displacement
float previoustotalAngle = 0; //for the display printing
unsigned long encoderTimer = 0;


void setup()
{
  delay(3000);
  Serial.begin(115200); //USB serial for debugging and communication
  Serial.println("STM32F401 + AS5600 + TMC2209 Evaluation board demo"); //DEMO message
  delay(2000);

  SPI_2.begin();  //Start SPI2 (LCD)
  u8g2.begin(); //Initialize LCD library
  u8g2.setContrast(80);
  u8g2.setFontMode(0);
  u8g2.clearBuffer();         // clear the internal memory
  //u8g2.setFont(u8g2_font_VCR_OSD_tf); // choose a suitable font - 13 size (Large)
  u8g2.setFont(u8g2_font_scrum_tf); //8 size (Normal)
  //X  Y   TEXT
  u8g2.drawStr(0, 8, "   Stepper motor"); // write something to the internal memory
  u8g2.drawStr(0, 22, "    Eval. board");
  u8g2.drawStr(0, 38, "  STM32F401CCU6");
  u8g2.drawStr(0, 55, "AS5600 + TMC2209");
  u8g2.sendBuffer(); // transfer internal memory to the display
  //-------------------------------------------------------------------------
  //-------------------------------------------------------------------------
  //TMC2209 setup
  Serial2.begin(115200);          // Hardware UART initialization (USB communication via the COM port)
  driver.beginSerial(115200);     // Driver UART initialization (UART to TMC2209)
  pinMode(EN_PIN, OUTPUT);    //Enable motor pin
  pinMode(STEP_PIN, OUTPUT);  //Step pin
  pinMode(DIR_PIN, OUTPUT);   //Dir pin
  pinMode(MS1, OUTPUT);       //Microstepping config pin 1
  pinMode(MS2, OUTPUT);       //Microstepping config pin 1
  digitalWrite(MS1, LOW);     //MS1 = 0
  digitalWrite(MS2, LOW);     //MS2 = 0; Notice: When UART is used, these pins are used for assigning an address to the driver
  digitalWrite(EN_PIN, LOW);  //Enable driver

  driver.begin();                 // Start the library
  driver.toff(5);                 // Enables driver in software - TOFF: slow decay time
  driver.rms_current(600);        // Set motor RMS current (RMS*sqrt(2) ~ max current consumption)
  driver.microsteps(8);          // Set microsteps to 1/8th

  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop

  //Accelstepper config (step/dir stepping)
  stepper.setMaxSpeed(2000); // 100mm/s @ 80 steps/mm
  stepper.setSpeed(800); // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(1000); // 2000mm/s^2
  stepper.setEnablePin(EN_PIN); //Pass the enable pin to the library
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs(); //Enable motor current

  //Read back a few settings from the driver to check if it works as we set it up...
  uint16_t msread = driver.microsteps();
  Serial.print(F("Read microsteps via UART to test UART receive : "));
  Serial.println(msread);
  delay(10);
  Serial.print(F("Read RMS via UART to test UART receive : "));
  Serial.println(driver.rms_current());
  delay(10);
  Serial.print(F("Read PWM via UART to test UART receive : "));
  Serial.println(driver.pwm_autoscale());
  //-------------------------------------------------------------------------
  //Config rotary encoder and buttons
  //-------------------------------------------------------------------------
  pinMode(UpButton, INPUT); //"Start"
  pinMode(DownButton, INPUT); //"Stop"
  pinMode(RotaryCLK, INPUT); //CLK
  pinMode(RotaryDT, INPUT); //DT
  pinMode(RotarySW, INPUT); //SW
  //Store the states of the encoder pins
  CLKPrevious = digitalRead(RotaryCLK);
  DTPrevious = digitalRead(RotaryDT);

  //Rotary encoder interrupt
  attachInterrupt(digitalPinToInterrupt(RotaryCLK), RotaryEncoder, CHANGE);
  //-------------------------------------------------------------------------
  //-------------------------------------------------------------------------

  Wire.begin(); //start i2C  //PB6 = SCL, PB7 = SDA
  Wire.setClock(800000L); //fast clock

  checkMagnetPresence(); //check the magnet (blocks until magnet is found)

  if (doIntro) //If we enabled this (doIntro = true) earlier in the code, it will move the motor in different ways
  {
    //Stepping the motor using the traditional STEP and DIR pins without library, just toggling the pins using the digitalWrite()
    //Print on the display
    u8g2.clearBuffer();
    u8g2.drawStr(0, 8, "      STEP-DIR"); // write something to the internal memory
    u8g2.drawStr(0, 22, "      stepping"); // write something to the internal memory
    u8g2.drawStr(0, 36, " by toggling pins"); // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Print on serial terminal
    Serial.println("Step-dir stepping with pin toggling");

    driver.microsteps(8); //Set microstepping ("most coarse" setting on TMC2209)

    //Go in one direction, 10k steps
    for (uint16_t i = 10000; i > 0; i--)
    {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(100);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(100);
    }
    
    shaft = !shaft; //Set opposite direction than previously
    driver.shaft(shaft); //Pass the direction (via UART!)

    delay(500); //0.5 s delay between changing directions

    driver.microsteps(16); //Set finer microstepping to demonstrate that the UART interface works well

    //Go 10k steps again, but with finer uStep and opposite direction
    for (uint16_t i = 10000; i > 0; i--)
    {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(100);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(100);
    }

    shaft = !shaft; //Change rotation direction again
    driver.shaft(shaft); //Push settings to driver via UART
    delay(500); //Wait 0.5 s

    driver.microsteps(32); //Apply even finer uStep

    //Go 10k steps with even less uStep
    for (uint16_t i = 10000; i > 0; i--)
    {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(100);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(100);
    }
    //You can recognize the pattern...
    shaft = !shaft;
    driver.shaft(shaft);

    delay(500);

    driver.microsteps(64);

    for (uint16_t i = 10000; i > 0; i--)
    {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(100);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(100);
    }
    shaft = !shaft;
    driver.shaft(shaft);
    delay(2000);
    //-----------------------------------------------------------------------------
    //STEP-DIR, but using the AccelStepper library
    Serial.println("Step-dir stepping using the AccelStepper library");
    //Print on the display
    u8g2.clearBuffer();
    u8g2.drawStr(0, 8, "      STEP-DIR"); // write something to the internal memory
    u8g2.drawStr(0, 22, "      stepping"); // write something to the internal memory
    u8g2.drawStr(0, 36, "   AccelStepper"); // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the display

    driver.microsteps(8);
    stepper.moveTo(8 * 200 * 10); //Set up an absolute target for the stepper motor

    while (stepper.distanceToGo() != 0) //While the target is not reached, run the motor
    {
      stepper.run(); //Run the motor with acceleration/deceleration
    }

    delay(2000);

    //-----------------------------------------------------------------------------
    Serial.println("UART stepping");

    //Print on the display
    u8g2.clearBuffer();
    u8g2.drawStr(0, 8, "         UART"); // write something to the internal memory
    u8g2.drawStr(0, 22, "      stepping"); // write something to the internal memory
    u8g2.drawStr(0, 40, "      TMC2209"); // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the display

    driver.microsteps(8); // 1/8th microstepping
    driver.VACTUAL(1600); //Set the motor speed to 2 turns/sec (1600 steps/s * 1/8 = 200 step/s)
    delay(5000); // Run the motor for 5 seconds
    driver.VACTUAL(0); //Stop the motor by setting its speed to zero
    shaft = !shaft; //Flip rotation direction
    driver.shaft(shaft);
    //Go the same in the other direction
    driver.VACTUAL(1600);
    delay(5000);
    driver.VACTUAL(0);

    delay(2000);

    stepper.setCurrentPosition(0); //Set the Accelstepper position to zero
    stepper.setSpeed(1600); //Set speed (the above line sets it to zero, so we must give a value)
    //-----------------------------------------------------------------------------------
    //Initializing AS5600
    Serial.println("Initializing AS5600");

    //Print on the display
    u8g2.clearBuffer();
    u8g2.drawStr(0, 18, "        AS5600"); // write something to the internal memory
    u8g2.drawStr(0, 32, "   initializing"); // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the display
  }
  //Read the zero position of the stepper motor shaft
  ReadRawAngle(); //make a reading so the degAngle gets updated
  startAngle = degAngle; //update startAngle with degAngle - for taring
  delay(2000);
}

void loop()
{

  CheckRotaryButton();

  CheckControlButtons();

  if(buttonControl == false) //Only run with AccelStepper (STEP-DIR) if the buttons are not driving the motor
  {
    if (stepper.distanceToGo() == 0)
    {
        stepper.disableOutputs();
        delay(100);
    }

    stepper.runToPosition(); //This blocks!
  }
  

  if (millis() - encoderTimer > 125) //125 ms will be able to make 8 readings in a sec which is enough for 60 RPM
  {
    ReadRawAngle(); //ask the value from the sensor
    correctAngle(); //tare the value
    checkQuadrant(); //check quadrant, check rotations, check absolute angular position
    updateLCD();

    encoderTimer = millis();
  }

}

void CheckControlButtons()
{
  //This is just a very coarse demo of using the buttons
  //The buttons are being polled in each loop() iteration and if they are pressed, the motor is moved
  //UpButton is positive (CW) rotation, DownBotton is negative (CCW)
  //This is just a demo that should be used individually, do not press the buttons while performing other things

  if (digitalRead(UpButton) == 0) //Start rotation
  {
    if (millis() - PushButtonTime > 1000) //1 sec debounce - I only debounce the start, because for stopping it doesn't matter
    {
    buttonControl = true; //setting this true does not let the motor to be controlled by the rotary encoder
    stepper.enableOutputs();
    shaft = !shaft;
    driver.shaft(shaft);
    driver.VACTUAL(800); //0.5 turn per second
    PushButtonTime = millis();
    }
  }

  if (digitalRead(DownButton) == 0) //Stop rotation
  {
    buttonControl = false; //We stop the button movement
    driver.VACTUAL(0); //Speed = 0 -> STOP
  }

}

void CheckRotaryButton()
{
  RotaryButtonValue = digitalRead(RotarySW); //read the button state

  if (RotaryButtonValue == 1) //if the button on the encoder was pressed
  {
    if (millis() - RotaryButtonTime > 1000) //1 sec debounce
    {
      Serial.println("Button pressed!");
      stepper.move(400 * rotaryCounter); //8 uStep * 50 step/turn (1.8 deg resolution, 1/4th turn)
      //The above command moves the shaft by 90 degrees for every 1 counter value
      //For example if you see "Encoder: 3" on the display, the shaft will move 270 degrees. 
      stepper.enableOutputs(); //Enable the driver
      RotaryButtonTime = millis();
    }
  }
}

void updateLCD()
{
  //Lazy implementation
  u8g2.setFontMode(1); //Transparent font - Next printing will overwrite the previous printing
  u8g2.drawStr(0, 10, "Total: "); // write something to the internal memory
  u8g2.drawStr(0, 24, "Absolute: ");
  u8g2.drawStr(0, 38, "Encoder: ");
  //-------------------------------------------
  u8g2.setCursor(60, 10);
  u8g2.print(totalAngle); //Absolute position of the encoder
  u8g2.setCursor(70, 24);
  u8g2.print(correctedAngle); //Tared angle measured from the starting position
  u8g2.setCursor(70, 38);
  u8g2.print(rotaryCounter); //Rotary encoder clicks
  u8g2.sendBuffer(); // transfer internal memory to the display
  u8g2.clearBuffer();
  //Note: print is used to print numerical values
  //wherease drawstr is used for string (text)
}

void RotaryEncoder() //Very simple and primitive rotary encoder interrupt code (but it works)
{
  CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
  
  // If last and current state of CLK are different, then a pulse occurred
  if (CLKNow != CLKPrevious && CLKNow == 1)
  {
    // If the DT state is different than the CLK state then
    // the encoder is rotating in A direction, so we increase
    if (digitalRead(RotaryDT) != CLKNow)
    {
      rotaryCounter++;
    }
    else //otherwise we decrease the value of the variable
    {
      rotaryCounter--;
    }
  }
  CLKPrevious = CLKNow;
}
//NOTE: The PEC11R does not work properly with this code, probably, because it is a 24 detent encoder
//The cheaper Aliexpress encoder works just fine, because the above code is suited for 12 detent encoder

//Below is the code for the AS5600.
//Relevant article and video: https://curiousscientist.tech/blog/as5600-magnetic-position-encoder
void ReadRawAngle()
{
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor

  while (Wire.available() == 0); //wait until it becomes available
  lowbyte = Wire.read(); //Reading the data after the request

  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);

  while (Wire.available() == 0);
  highbyte = Wire.read();

  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in

  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625;

  Serial.print("Deg angle: ");
  Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle

}

void correctAngle()
{
  //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if (correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
  {
    correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
  //Serial.print("Corrected angle: ");
  //Serial.println(correctedAngle, 2); //print the corrected/tared angle
}

void checkQuadrant()
{
  /*
    //Quadrants:
    4  |  1
    ---|---
    3  |  2
  */

  //Quadrant 1
  if (correctedAngle >= 0 && correctedAngle <= 90)
  {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if (correctedAngle > 90 && correctedAngle <= 180)
  {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if (correctedAngle > 180 && correctedAngle <= 270)
  {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if (correctedAngle > 270 && correctedAngle < 360)
  {
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if (quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if (quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if (quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant

  }
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns * 360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}

void checkMagnetPresence()
{
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while ((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while (Wire.available() == 0); //wait until it becomes available
    magnetStatus = Wire.read(); //Reading the data after the request

    Serial.print("Magnet status: ");
    Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)
  }

  //Status register output: 0 0 MD ML MH 0 0 0
  //MH: Too strong magnet - 100111 - DEC: 39
  //ML: Too weak magnet - 10111 - DEC: 23
  //MD: OK magnet - 110111 - DEC: 55

  Serial.println("Magnet found!");
  delay(1000);
}
