#include <movingAvg.h>


// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      200         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(2,3);        // RX, TX

movingAvg steerAvg(15);
movingAvg speedAvg(15);
int8_t direction = 0;
uint8_t buttonPin = 12;
uint8_t dipBeamPin = 11;
uint8_t highBeamPin = 10;
uint8_t relay3Pin = 9;
uint8_t relay4Pin = 8;
bool buttonPressed = false;
uint32_t buttonPressedMillis = 0;
uint8_t headlightMode = 0; //0 - off, 1 - dip, 2 - main

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  steerAvg.begin();
  speedAvg.begin();
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(dipBeamPin, OUTPUT);
  digitalWrite(dipBeamPin, HIGH);
  pinMode(highBeamPin, OUTPUT);
  digitalWrite(highBeamPin, HIGH);
  pinMode(relay3Pin, OUTPUT);
  digitalWrite(relay3Pin, HIGH);
  pinMode(relay4Pin, OUTPUT);
  digitalWrite(relay4Pin, HIGH);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

uint16_t speedPotValue = 0;
int16_t speedValue = 0;

uint16_t steerPotValue = 0;
int16_t steerValue = 0;

void loop(void)
{ 
  speedPotValue = speedAvg.reading(analogRead(A0));
  steerPotValue = steerAvg.reading(analogRead(A1));

  if(speedPotValue > 526) direction = 1;
  if(speedPotValue < 500) direction = -1;

  if(direction == 1) {
    speedValue = map(speedPotValue, 526, 1023, 0, -1000);
  }

  if(direction == -1) {
    speedValue = map(speedPotValue, 500, 0, 0, 500);
  }
  
  if(speedPotValue > 526 || speedPotValue < 500) {
    //speedValue = map(speedPotValue, 0, 1023, 500, -500);
  } else {
    speedValue = 0;
    direction = 0;
  }

  if(steerPotValue > 526 || steerPotValue < 500) {
    steerValue = map(steerPotValue, 0, 1023, 800, -800);
  } else {
    steerValue = 0;
  }
  Serial.print("Speed value: "); Serial.print(speedValue, DEC); Serial.print(", Steer value: "); Serial.print(steerValue, DEC); Serial.print(", Direction: "); Serial.println(direction, DEC);
  
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  Send(steerValue, speedValue);

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
  delay(1);

  if(digitalRead(buttonPin) == LOW) {
    buttonPressed = true;
    buttonPressedMillis = millis();
  }

  if(digitalRead(buttonPin) == HIGH && buttonPressed && millis() > buttonPressedMillis + 20) {
    buttonPressed = false;
    if(headlightMode == 0) {
      headlightMode = 1;
      digitalWrite(dipBeamPin, LOW);
    } else if(headlightMode == 1) {
      headlightMode = 2;
      digitalWrite(highBeamPin, LOW);
    } else if(headlightMode == 2) {
      headlightMode = 0;
      digitalWrite(highBeamPin, HIGH);
      digitalWrite(dipBeamPin, HIGH);
    }
  }
}

// ########################## END ##########################
