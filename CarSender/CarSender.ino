/*
 * This example shows how read arcade buttons and PWM the LEDs on the Adafruit Arcade QT!
 */

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/************ Car ***************/
#define CAR_ID 'C' // C for Cruz, L for Lightning

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// Where to send packets to!
#define DEST_ADDRESS 1
// change addresses for each client board, any number :)
#define MY_ADDRESS 2

#if defined(__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS 8
#define RFM69_INT 7
#define RFM69_RST 4
#define LED 13
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4
#define LED 13
#endif

#if defined(__AVR_ATmega328P__) // Feather 328P w/wing
#define RFM69_INT 3             //
#define RFM69_CS 4              //
#define RFM69_RST 2             // "A"
#define LED 13
#endif

#if defined(ESP8266) // ESP8266 feather w/wing
#define RFM69_CS 2   // "E"
#define RFM69_IRQ 15 // "B"
#define RFM69_RST 16 // "D"
#define LED 0
#endif

#if defined(ESP32)   // ESP32 feather w/wing
#define RFM69_RST 13 // same as LED
#define RFM69_CS 33  // "B"
#define RFM69_INT 27 // "A"
#define LED 13
#endif

#define VBATPIN A9
#define LED 13
#define BUTTON_PIN 12
#define BUTTON_LED_PIN A0

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0; // packet counter, we increment per xmission

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

#define BOARD_1 0x3A
#define BOARD_2 0x3B

#define SWITCH1 18 // PA01
#define SWITCH2 19 // PA02
#define SWITCH3 20 // PA03
#define SWITCH4 2  // PA06

#define SWITCH5 18
#define SWITCH6 19
#define SWITCH7 20
#define SWITCH8 2

#define PWM1 12 // PC00
#define PWM2 13 // PC01
#define PWM3 0  // PA04
#define PWM4 1  // PA05

#define PWM5 12
#define PWM6 13
#define PWM7 0
#define PWM8 1

#define SPEED 20

Adafruit_seesaw ss1;
Adafruit_seesaw ss2;

int power[8] = {PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8};
int switches[8] = {SWITCH1, SWITCH2, SWITCH3, SWITCH4, SWITCH5, SWITCH6, SWITCH7, SWITCH8};
int brightness[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int states[8] = {0, 0, 0, 0, 0, 0, 0, 0};

#define TEN_SEC 10000
unsigned long lastSentMS = 0;

void setup()
{
  Serial.begin(115200);
  //  while (!Serial) delay(10);   // wait until serial port is opened

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init())
  {
    Serial.println("RFM69 radio init failed");
    while (1)
      ;
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ))
  {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = {0x02, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");
  Serial.print((int)RF69_FREQ);
  Serial.println(" MHz");

  // initialize the first board
  if (!ss1.begin(BOARD_1))
  {
    Serial.println(F("BOARD_1 seesaw not found!"));
    while (1)
      delay(10);
  }

  // initialize the second board
  if (!ss2.begin(BOARD_2))
  {
    Serial.println(F("BOARD_2 seesaw not found!"));
    while (1)
      delay(10);
  }

  Serial.println(F("seesaw started OK!"));

  for (int i = 0; i < 8; i++)
  {
    Serial.println(F("set up board"));
    int switchPin = switches[i];
    if (i < 4)
    {
      ss1.pinMode(switchPin, INPUT_PULLUP);
    }
    else
    {
      ss2.pinMode(switchPin, INPUT_PULLUP);
    }
  }

  Serial.println(F("Button pins set up!"));
}

int startUpCnt = 0;
int startUpDelay = 0;

void loop()
{
  startUpDelay++;
  if (startUpCnt < 20 && startUpDelay < 100)
  {
    startUpCnt++;
    int randIndex = (int)random(0, 8);
    brightness[randIndex] = 255;
  }

  unsigned long ms = millis();
  if ((ms - lastSentMS) > TEN_SEC)
  {
    lastSentMS = ms;
    SendMessage("OC");
  }

  for (int i = 0; i < 8; i++)
  {
    int powerPin = power[i];
    int switchPin = switches[i];
    int state = states[i];

    int buttonVal = 0;
    if (i < 4)
    {
      buttonVal = ss1.digitalRead(switchPin);
    }
    else
    {
      buttonVal = ss2.digitalRead(switchPin);
    }

    bool isButtonPressed = (!buttonVal && buttonVal != state);

    // read the button values and set the brightness
    if (isButtonPressed)
    {
      Serial.println(F("Button Pressed"));
      brightness[i] = 255;
    }

    // dim the LEDs over time
    if (brightness[i] > SPEED)
    {
      brightness[i] = brightness[i] - SPEED;
    }
    else if (brightness[i] > 0)
    {
      brightness[i] = 0;
    }

    // apply the brightness to the LEDs
    if (i < 4)
    {
      ss1.analogWrite(powerPin, brightness[i]);
    }
    else
    {
      ss2.analogWrite(powerPin, brightness[i]);
    }

    // send the radio message last so the button response to the press
    if (isButtonPressed)
    {
      ButtonPress(i);
    }

    states[i] = buttonVal;
  }

  delay(10);
}

void ButtonPress(int button)
{
  String message = CAR_ID + (String)button;
  char msg[message.length() + 1];
  message.toCharArray(msg, message.length() + 1);
  SendMessage(msg);
}

void SendMessage(char radiopacket[1])
{
  itoa(packetnum++, radiopacket + 13, 10);
  Serial.print("Sending ");
  Serial.println(radiopacket);

  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS))
  {
    Serial.println("Sent!");
  }
  else
  {
    Serial.println("Sending failed (no ack)");
  }
}