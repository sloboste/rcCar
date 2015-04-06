#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

unsigned long prevMili = 0;
unsigned long interval = 10;
int LED_state = LOW;

const uint8_t PWM_ADDR = 0x40;
const uint8_t PWM_STEER = 0;
const uint8_t PWM_MOTOR = 1;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_ADDR);

void setup()
{
    Serial.begin(9600);
    //pinMode(3, INPUT);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    
    //Wire.begin(0x34);
    //Wire.onRequest(request);
    //Wire.onReceive(receive);

    pwm.begin();
    pwm.setPWMFreq(72.0);

    // pinNum, offTime(0-4095), invert
    pwm.setPin(PWM_STEER, (4095/2), false); 
    pwm.setPin(PWM_MOTOR, (4095/2), false); 
    
    Serial.println("done setup");
    return;
}

unsigned int count = 0;
bool desc = false;
//const int PW_MIN = 1140; //us
//const int PW_MAX = 2720; //us
const int CNT_MIN = 350;
const int CNT_MAX = 570;
 

//uint32_t x = 447;

void loop()
{    
    // Blink led to indicate that the board is working
    unsigned long curMili = millis();
    if ( (curMili - prevMili) > interval) { // blink led
        if (LED_state == LOW) {
            LED_state = HIGH;
        } else {
            LED_state = LOW;
            
            // pwm stuff
            if (desc) {
                if (count == 0) {
                    desc = false;
                    ++count;
                } else {
                    --count;
                }
            } else {
                if (count == 100) {
                    desc = true;
                    --count;
                } else {
                    ++count;
                }
            }

            int n = map(count, 0, 100, CNT_MIN, CNT_MAX); 
            pwm.setPin(PWM_STEER, n, false);
            pwm.setPin(PWM_MOTOR, n, false);
            /*
            pwm.setPin(PWM_STEER, x, false);
            pwm.setPin(PWM_MOTOR, x, false);
            */

        }
        prevMili = curMili;
        digitalWrite(13, LED_state); 
    } else if (prevMili > curMili) { // overflow occured
        prevMili = 0;
    } 

/*
    if (Serial.available() > 0) {
        char c = Serial.read();
        Serial.println(c);
        if (c == '+') {
            x += 10;
        } else if (c == '-') {
            x -= 10;
        }
        Serial.print("x = "); Serial.println(x);
    }
    */

    return;
}

void request()
{
    Serial.println("request");
    Wire.write(0xAD);
}

void receive(int howMany)
{
    Serial.print("receive ");
    Serial.print(howMany);
    Serial.println(" bytes");
    for (int i = 0; i < howMany; ++i) {
        Serial.print("byte number ");
        Serial.print(i);
        Serial.print(" = ");
        int x = Wire.read();
        Serial.println(x);
    }
}

