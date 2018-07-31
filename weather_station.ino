#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <Wire.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_VEML6070.h>
/************ Radio Setup ***************/

// Change to 915.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     2

//How many loops to make before restarting loop counter
#define MAX_LOOPS 114

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
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

#define VBATPIN A9
#define RANDOM_NOISE A5
#define RAIN_GAUGE_PIN 0

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

//Driver for humidity sensor
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

//Driver for UV sensor
Adafruit_VEML6070 uv = Adafruit_VEML6070();


int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 915.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  //Seed the random number generator, this is a little goofy, took it from the web, not sure what I think about it yet.
  unsigned long seed = 0;
  for (int i=0; i<32; i++)
  {
    seed = seed | ((analogRead(RANDOM_NOISE) & 0x01) << i);
  }
  randomSeed(seed);

  //Increase the retries from the default 3 to 10
  rf69_manager.setRetries(10);


  if (!htu.begin()) {
    Serial.println("Couldn't find sensor!");
    while (1);
  }

  pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), blink_interrupt, FALLING);

  //Init the UV sensor and then put it to sleep
  uv.begin(VEML6070_4_T);
  uv.sleep(true);
}


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";
volatile bool rain_flag = false;
uint8_t loop_counter = 0;

void loop() {
  delay(500);  // Wait 1 second between transmits, could also 'sleep' here!
  //Watchdog.sleep(500);

  //Start a conversion a few loops before we send our packet to give it time to complete and for us to read
  if (MAX_LOOPS - 3 == loop_counter) 
  {
    //Wake up the sensor
    uv.sleep(false);
    //Start reading
    uv.begin(VEML6070_4_T);
  }
  
  
  if (rain_flag)
  {
    rain_flag = false;
    
    Serial.println();
    Serial.println("TIP");
    Serial.println();

    //Initialize the payload
    byte radiopacket[5] = "";
    generate_unique_id(radiopacket);
    radiopacket[4] = 1;

    // Send a message to the DESTINATION!
    if (rf69_manager.sendtoWait((uint8_t *)radiopacket, 5, DEST_ADDRESS)) {
      //Blink if we got an ACK
      Blink(LED, 80, 2);
    } else {
      Serial.println("Sending failed (no ack)");
      Blink(LED, 20, 8);
    }
    rf69.sleep();
    attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), blink_interrupt, FALLING);   
  }

  //we have a 500ms delay, executed 120 times would be 1 min, however, transmit takes some time, and flashing
  //the heartbeat takes 50ms every other loop so to get us close to a one minute send interval we shorten our
  //loop count some
  if (loop_counter >= MAX_LOOPS)
  {
    loop_counter = 0;
    
    //Read the battery voltage   
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    Serial.print("VBat: " ); Serial.println(measuredvbat);

    //Read the temp and humidity
    float htu_temp = htu.readTemperature();
    float htu_humidity = htu.readHumidity();
    Serial.print("Temp: "); Serial.print(htu_temp);
    Serial.print("\t\tHum: "); Serial.println(htu_humidity);

    //Read the UV sensor
    uint16_t uv_val =  uv.readUV();
    Serial.print("UV:"); Serial.println(uv_val);
    
  
    //Initialize the payload
    byte radiopacket[11] = "";
  
    generate_unique_id(radiopacket);
  
    Serial.print("ID Bytes: "); Serial.print((byte)radiopacket[0]); Serial.print(" "); Serial.print((byte)radiopacket[1]);Serial.print(" ");Serial.print((byte)radiopacket[2]);Serial.print(" ");Serial.println((byte)radiopacket[3]);
    
    //Break the battery voltage up into one byte for the 1's place
    radiopacket[4] = (byte) measuredvbat;
    //And 1 byte for the tenths and hundredths
    radiopacket[5] = (byte)((measuredvbat - radiopacket[4]) * 100);
  
    radiopacket[6] = (byte) htu_temp;
    radiopacket[7] = (byte)((htu_temp - radiopacket[6]) * 100);
  
    radiopacket[8] = (byte) htu_humidity;
    radiopacket[9] = (byte)((htu_humidity - radiopacket[8])*100);
    
    //Also send back an indicator of retransmissions
    radiopacket[10] = (byte) rf69_manager.retransmissions();
    //Reset the counter
    rf69_manager.resetRetransmissions();
    
      
    // Send a message to the DESTINATION!
    if (rf69_manager.sendtoWait((uint8_t *)radiopacket, 11, DEST_ADDRESS)) {
      //Blink if we got an ACK
      Blink(LED, 40, 4);
    } else {
      Serial.println("Sending failed (no ack)");
      Blink(LED, 20, 8);
    }
    //Put the radio to sleep
    rf69.sleep();
    
    //Put the UV sensor to sleep
    uv.sleep(true);
  }

  loop_counter++;

  //Heartbeat
  if(loop_counter % 2 == 0){
    Serial.println(loop_counter);
    single_blink(LED, 50);
  }
  
}

void generate_unique_id(byte* packet){
  //Create a random 32bit ID to send with this packet
    for (int i = 0; i < 4; i++)
    {
      packet[i] = (byte) ((((byte)analogRead(RANDOM_NOISE) & 0xFF)) ^ ((byte) random(256)));
    }
}

void blink_interrupt() {
  if (digitalRead(RAIN_GAUGE_PIN) == LOW)
  {
    rain_flag = true;
    //Disable the interrupt to avoid bouncing
    detachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN));
  }
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

void single_blink(byte PIN, byte DELAY_MS) {
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

