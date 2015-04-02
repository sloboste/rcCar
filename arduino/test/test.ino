#include <Wire.h>

unsigned long prevMili = 0;
unsigned long interval = 500;
int LED_state = LOW;

void setup()
{
    Serial.begin(9600);
    pinMode(3, INPUT);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    
    Wire.begin(0x34);
    Wire.onRequest(request);
    Wire.onReceive(receive);
    
    Serial.println("done setup");
    return;
}

void loop()
{    
    // Blink led to indicate that the board is working
    unsigned long curMili = millis();
    if ( (curMili - prevMili) > interval) { // blink led
        if (LED_state == LOW) {
            LED_state = HIGH;
        } else {
            LED_state = LOW;
        }
        prevMili = curMili;
        digitalWrite(13, LED_state); 
    } else if (prevMili > curMili) { // overflow occured
        prevMili = 0;
    } 
    
    return;
}

void request()
{
    Serial.println("request");
    Wire.write(0xAD);
}

void receive(int howMany)
{
    Serial.println("receive");
    int x = Wire.read();
    Serial.println(x);
}

