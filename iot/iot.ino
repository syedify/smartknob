
/*
  Getting Started example sketch for nRF24L01+ radios
  This is an example of how to send data from one node to another using data structures
  Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include <Arduino.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

byte addresses[][4] = {"Hub", "Dev"};


// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output


// SOFTWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins that will be used for 'SW' serial.
// You should use this option if you are connecting the UART Friend to an UNO
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SWUART_RXD_PIN       5    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       4   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         3   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         -1   // Optional, set to -1 if unused


// HARDWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the HW serial port you are using. Uncomment
// this line if you are connecting the BLE to Leonardo/Micro or Flora
// ----------------------------------------------------------------------------------------------
#ifdef Serial1    // this makes it not complain on compilation if there's no Serial1
#define BLUEFRUIT_HWSERIAL_NAME      Serial1
#endif


// SHARED UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following sets the optional Mode pin, its recommended but not required
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_UART_MODE_PIN        12    // Set to -1 if unused


// SHARED SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for HW and SW SPI communication.
// SCK, MISO and MOSI should be connected to the HW SPI pins on the Uno when
// using HW SPI.  This should be used with nRF51822 based Bluefruit LE modules
// that use SPI (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4    // Optional but recommended, set to -1 if unused

// SOFTWARE SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for SW SPI communication.
// This should be used with nRF51822 based Bluefruit LE modules that use SPI
// (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_SCK              13
#define BLUEFRUIT_SPI_MISO             12
#define BLUEFRUIT_SPI_MOSI             11
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

#define ROTATE_L 0x02
#define ROTATE_R 0x04
#define PUSH     0x06
#define SELECT   0x0F
#define WAKEUP   0X08
#define DIAL_1   0x10
#define DIAL_2   0x20

#define ROLE_HUB 1
#define ROLE_DEV 0

#define TIMEOUT_100us 1000000
#define TIMEOUT_50us   500000

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
//RF24 radio(7, 8);
RF24 radio(7, 9);
/**********************************************************/


// Used to control whether this node is sending or receiving
//bool role = ROLE_HUB;
bool role = ROLE_DEV;
uint8_t id = DIAL_2;
char inputs[BUFSIZE + 1];
/**
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/
struct dataStruct {
  unsigned long _micros;
    uint32_t value;
} myData;


uint32_t gen_packet(uint32_t id, uint32_t cmd)
{
  uint32_t p;
  p = id | cmd;
  Serial.println("Generating command " + p);
  return p;
}


/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while ( (!Serial.available()) && !timeout.expired() ) {
    delay(1);
  }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count = 0;
  do
  {
    count += Serial.readBytes(buffer + count, maxSize);
    delay(2);
  } while ( (count < maxSize) && (Serial.available()) );

  return true;
}

void ble_get_data ()
{
  // Check for user input
  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }

  if (ble.buffer == "w1")
  {
      myData.value = gen_packet(DIAL_1, ROTATE_L);
  }
  else if (ble.buffer == "w0")
  {
      myData.value = gen_packet(DIAL_1, ROTATE_R); 
  }
  else if (ble.buffer == "s1")
  {
    myData.value = gen_packet(DIAL_2, ROTATE_L);
  }
  else if (ble.buffer == "s0")
  {
    myData.value = gen_packet(DIAL_2, ROTATE_R);
  }
  else
  {
    myData.value = gen_packet(DIAL_1, SELECT);
  }
  // Some data was found, its in the buffer
  Serial.print(F("[Recv] "));
  Serial.println(ble.buffer);
  ble.waitForOK();
}

void setup() {
  //uint8_t role = ROLE_HUB;
  Serial.begin(9600);
  //Serial.println(F("RF24/examples/GettingStarted_HandlingData"));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.

  /* Set the RF power of the device */
  if (role == ROLE_HUB)
  { /* Initialise the module */

//    Serial.print(F("Initialising the Bluefruit LE module: "));
//
//    if ( !ble.begin(VERBOSE_MODE) )
//    {
//      error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
//    }
//    Serial.println( F("OK!") );
//
//    if ( FACTORYRESET_ENABLE )
//    {
//      /* Perform a factory reset to make sure everything is in a known state */
//      Serial.println(F("Performing a factory reset: "));
//      if ( ! ble.factoryReset() ) {
//        error(F("Couldn't factory reset"));
//      }
//    }
//
//    /* Disable command echo from Bluefruit */
//    ble.echo(false);
//
//    Serial.println("Requesting Bluefruit info:");
//    /* Print Bluefruit information */
//    ble.info();
//
//    Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
//    Serial.println(F("Then Enter characters to send to Bluefruit"));
//    Serial.println();
//
//    ble.verbose(false);  // debug info is a little annoying after this point!
//
//    /* Wait for connection */
//    while (! ble.isConnected()) {
//      delay(500);
//    }
//
//    // LED Activity command is only supported from 0.6.6
//    if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
//    {
//      // Change Mode LED Activity
//      Serial.println(F("******************************"));
//      Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
//      ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
//      Serial.println(F("******************************"));
//    }
    Serial.println(F("This is the HUB"));
    radio.begin();
    //    btle.begin("BLE_Hub");
    radio.setPALevel(RF24_PA_LOW);
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
  }
  else
  {
    Serial.println(F("This is the DIAL"));
    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
    id = DIAL_1;
  }


  // Open a writing and reading pipe on each radio, with opposite addresses
  //  if (radioNumber) {
  //    radio.openWritingPipe(addresses[1]);
  //    radio.openReadingPipe(1, addresses[0]);
  //  } else {
  //    radio.openWritingPipe(addresses[0]);
  //    radio.openReadingPipe(1, addresses[1]);
  //  }

  //myData.value = 1.22;
  // Start the radio listening for data
  //radio.startListening();
}

uint8_t test = 1;
uint8_t inst = 0;
void rf_hub_execute()
{
  Serial.print("HUB ");

  radio.stopListening();                                    // First, stop listening so we can talk.

  myData.value = (test << 4);
  test++;
  
  switch (inst)
  {
    case 1:
      myData.value |= PUSH;
      break;
    case 2:
      myData.value |= ROTATE_R;
      break;
    case 3:
      myData.value |= ROTATE_L;
      break;
    default:
      myData.value |= SELECT;
      break;
  }

  if(++inst > 4)
  {
    inst = 1;
  }
  
  if (test > 2)
  {
    test = 1;
  }

  Serial.println(F("Now sending"));

  myData._micros = micros();
  
  Serial.print ("GENERATED PACKET: ");
  Serial.println (myData.value, HEX);
  if (!radio.write( &myData, sizeof(myData) )) {
    //Serial.println(F("failed"));
  }

  radio.startListening();                                    // Now, continue listening

  unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not

  while ( ! radio.available() ) {                            // While nothing is received
    if (micros() - started_waiting_at > (TIMEOUT_100us * 2) ) {         // If waited longer than 200ms, indicate timeout and exit while loop
      timeout = true;
      break;
    }
  }

  if ( timeout ) {                                            // Describe the results
    Serial.println(F("Failed, response timed out."));
  } else {
    // Grab the response, compare, and send to debugging spew
    radio.read( &myData, sizeof(myData) );

    unsigned long time = micros();
    // Spew it
    Serial.print(F("Sent "));
    Serial.print(time);
    Serial.print(F(", Got response "));
    Serial.print(myData._micros);
    Serial.print(F(", Round-trip delay "));
    Serial.print(time - myData._micros);
    Serial.print(F(", ID "));
    Serial.println(myData.value >> 4, HEX);
    Serial.print(F(", Command "));
    Serial.println(myData.value & 0x0F, HEX);

    switch (myData.value & 0xF0)
    {
      case DIAL_1:
        Serial.println("Device 1 ACK");
        break;

      case DIAL_2:
        Serial.println("Device 2 ACK");
        break;

      default:
        Serial.println("NOISY PACKET");
        break;
    }
  }
  // Try again 500ms later
  delay(2000);
}


void rf_dev_execute()
{

  /****************** Pong Back Role ***************************/

  //Serial.print ("DEV ");
  if ( radio.available()) {
    // Variable for the received timestamp
    while (radio.available()) {                          // While there is data ready
      radio.read( &myData, sizeof(myData) );             // Get the payload
    }

    radio.stopListening();                               // First, stop listening so we can talk
    //myData.value += 0.01;                                // Increment the float value
    radio.write( &myData, sizeof(myData) );              // Send the final one back.
    radio.startListening();                              // Now, resume listening so we catch the next packets.
    Serial.print(F("Sent response "));
    Serial.print(myData._micros);
    Serial.print(F(" : "));
    Serial.println(myData.value, HEX);

//    if ((myData.value >> 4) == (id >> 4))
//    {
      switch (myData.value & 0xF)
      {
        case ROTATE_L:
          Serial.println("ROATING DIAL LEFT");
        break;

        case ROTATE_R:
          Serial.println("ROATING DIAL RIGHT");
        break;

        case PUSH:
          Serial.println("PUSHING DIAL");
        break;

        case SELECT:
          Serial.println("SELECT DIAL");
        break;
      }
    //}
  }
  delay(1000);
}

void loop() {


  /****************** Ping Out Role ***************************/
  if (role == ROLE_HUB)  {
    //ble_get_data();
    myData.value = 0x1F;
    radio.startListening();
    rf_hub_execute();
    radio.stopListening();
  }
  else if ( role == ROLE_DEV ) {
    radio.startListening();
    rf_dev_execute();
    radio.stopListening();
  }

  /****************** Change Roles via Serial Commands ***************************/

  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'D' && role == ROLE_HUB ) {
      Serial.print(F("*** CHANGING TO DEVICE ROLE -- PRESS 'D' TO SWITCH BACK"));
      role = ROLE_DEV;                  // Become the primary transmitter (ping out)

    } else if ( c == 'H' && role == ROLE_DEV ) {
      Serial.println(F("*** CHANGING TO HUB ROLE -- PRESS 'H' TO SWITCH BACK"));
      role = ROLE_HUB;                // Become the primary receiver (pong back)
      radio.startListening();

    }
  }
} // Loop
