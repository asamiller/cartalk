#include <Adafruit_Soundboard.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <SoftwareSerial.h>

/************ Car ***************/
#define CAR_ID 'L'

/************ SFX Setup ***************/
// Connect to the RST pin on the Sound Board
#define SFX_RST 5
Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial1, NULL, SFX_RST);

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// who am i? (server address)
#define MY_ADDRESS 1

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

/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

/* WICED Feather w/wing
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

// Store the last online message received times
unsigned long lastSentMSLightning = 0;
unsigned long lastSentMSCruz = 0;

int16_t packetnum = 0; // packet counter, we increment per xmission

#define TWENTY_SEC 20000
bool isCruzOnline()
{
  return (millis() - lastSentMSCruz) < TWENTY_SEC;
}
bool isLightningOnline()
{
  return (millis() - lastSentMSLightning) < TWENTY_SEC;
}
bool areBothCarsOnline()
{
  return isLightningOnline() && isCruzOnline();
}

void setup()
{
  Serial.begin(115200);
  //  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 RX Test!");
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

  // SFX
  Serial1.begin(9600);
  if (!sfx.reset())
  {
    Serial.println("Not found");
    while (1)
      ;
  }
  Serial.println("SFX board found");
}

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop()
{
  if (rf69_manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from))
    {
      buf[len] = 0; // zero out remaining string

      Serial.print("Got packet from #");
      Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.print("] : ");
      Serial.println((char *)buf);

      char prefix = (char *)buf[0];
      char message = (char *)buf[1];

      Serial.print("prefix ");
      Serial.print(prefix);

      Serial.print(" message ");
      Serial.println(message);

      // If we get an online message, save the timestamp so we can know the car is online
      if (prefix == 'O')
      {
        if (message == 'L')
        {
          lastSentMSLightning = millis();
        }
        else if (message == 'C')
        {
          lastSentMSCruz = millis();
        }

        Blink(LED, 10, 2);
        return;
      }

      // Both cars are online, so play the joint sound effects
      if (areBothCarsOnline())
      {
        char filename[12];
        String track = String(message) + "BOTH000OGG";
        track.toCharArray(filename, 12);

        if (sfx.playTrack(filename))
        {
          Serial.print("playing both cars ");
          Serial.println(filename);
          Blink(LED, 10, 2);
          return;
        }
        else
        {
          Serial.print("Failed to play track name ");
          Serial.println(filename);
        }
      }

      // Play this car's sound effects
      if (prefix == CAR_ID)
      {
        int n = String(message).toInt();

        if (sfx.playTrack((uint8_t)n))
        {
          Serial.print("playing track ");
          Serial.println(n);
          Blink(LED, 10, 2);
          return;
        }
        else
        {
          Serial.print("Failed to play track number ");
          Serial.println(n);
        }
      }
    }
  }
}

void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i = 0; i < loops; i++)
  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}
