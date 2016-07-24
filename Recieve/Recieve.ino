
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);

const uint64_t rxAddr[6] = {1};

void setup()
{
 while (!Serial);
 Serial.begin(9600);
 
 radio.begin();
 radio.openReadingPipe(0, rxAddr[0]);
 
 radio.startListening();
}

void loop()
{
 if (radio.available())
 {
   char text[32] = {0};
   radio.read(&text, sizeof(text));
   radio.printDetails();
   Serial.println(text);
 }
}

