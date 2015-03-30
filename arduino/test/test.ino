#include <Servo.h>

unsigned long prevMili = 0;
unsigned long interval = 500;
int LED_state = LOW;
Servo serv;

void setup()
{
    Serial.begin(9600);
    pinMode(3, INPUT);
    pinMode(9, OUTPUT); // test
    serv.attach(9);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    return;
}

void loop()
{
    // Slow it down
    //delay(1000);

    // Read in 
    unsigned int pw = pulseIn(3, HIGH);
    unsigned char dc = map(pw, 1140, 2720, 0, 100); // Not really dutycycle
    Serial.print("in = ");
    Serial.println((float) pw/1000.0); 

    // Write out
    pw = map(dc, 0, 100, 1140, 2720);
    serv.writeMicroseconds(pw);
    //Serial.print("out = ");
    //Serial.println((float) pw/1000.0); 
   
   /* 
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
   */ 
    return;
}
