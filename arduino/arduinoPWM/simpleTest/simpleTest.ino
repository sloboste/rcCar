#include <Wire.h>

const unsigned char PIN_LED = 13;

void setup() 
{
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
}

void loop()
    digitalWrite(PIN_LED, HIGH);
    delay(500);//ms
    digitalWrite(PIN_LED, LOW);
    delay(500);//ms
}

