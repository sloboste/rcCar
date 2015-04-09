#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

unsigned long prevMili = 0;
unsigned long interval = 10;
int LED_state = LOW;

const uint8_t PWM_ADDR = 0x40;
const uint8_t PWM_STEER = 0;
const uint8_t PWM_MOTOR = 1;
const uint16_t CNT_NEUTRAL = 447;
const uint16_t CNT_MIN = 350;
const uint16_t CNT_MAX = 570;
uint16_t count = CNT_NEUTRAL;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_ADDR);

void setup()
{
    Serial.begin(9600);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    pwm.begin();
    pwm.setPWMFreq(72.0);

    // pinNum, offTime(0-4095), invert
    pwm.setPin(PWM_STEER, (4095/2), false); 
    pwm.setPin(PWM_MOTOR, (4095/2), false); 
    
    Serial.println("done setup");
    return;
}

void loop()
{    

    // Write to steer servo
    int n = map(count, 0, 100, CNT_MIN, CNT_MAX); 
    pwm.setPin(PWM_STEER, n, false);
    pwm.setPin(PWM_MOTOR, n, false);


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

void serialEvent()
{
    char cmd;
    char dir;
    uint8_t percent;
    String responseStr;
    bool success = true;

    // Read in command type
    if (Serial.available() > 0) {
        cmd = Serial.read();
        Serial.println(cmd); // DEBUGGING FIXME
    }
    // Check command type
    if (cmd == 'A') { // Set mode idle
        //mode = MODE_IDLE;
        responseStr = "Set mode idle";             
    } else if (cmd == 'B') { // Set mode rc
        //mode = MODE_RC;
        responseStr = "Set mode rc";             
    } else if (cmd == 'C') { // Set mode rpi
        //mode = MODE_RPI;
        responseStr = "Set mode rpi";             
    } else if (cmd == 'D') { // Set steer dir, % 
        if (Serial.available () >= 3) {
            dir = Serial.read();
            percent = (uint8_t) Serial.read(); 
            responseStr = "Set steer";             
        } else {
            responseStr = "X";
        }
    } else if (cmd == 'E') { // Set motor dir, %
        if (Serial.available () >= 3) {
            dir = Serial.read();
            percent = (uint8_t) Serial.read(); 
            responseStr = "Set motor";             
        } else {
            responseStr = "X";
        }
    } else if (cmd == 'F') { // Return steer dir, %
        // FIXME
        responseStr = "Return steer";             
    } else if (cmd == 'G') { // Return motor dir, %
        // FIXME
        responseStr = "Return motor";             
    } else { // Invalid cmd
        responseStr = "X";
    }

    // Send response
    Serial.println(responseStr);

    return;
}

