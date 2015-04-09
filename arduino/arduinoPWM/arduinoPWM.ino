/* Arduino PWM 
 * Microcomputer-Controlled Car Project
 * University of Michigan - Tilbury Research Group
 * Author: Steven Sloboda
 * Version: 0.3
 */


#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// 7 bit I2C address of PWM module (0x40 is default)
static const uint8_t PWM_I2C_ADDR = 0x40; 

// 8-bit identification number (arbitrarily assigned) of Arduino
static const uint8_t SELF_CHIP_ID = 0xAD; 

// Pins used on  the Arduino
//static const unsigned char PIN_I2C_SDA   = A4; // SDA 
//static const unsigned char PIN_I2C_SCL   = A5; // SCL
static const uint8_t PIN_PWM_IN_S  = 3; // Receiver channel 1 
static const uint8_t PIN_PWM_IN_M  = 5; // Receiver channel 2
// Data line close to the red button on the receiver...

// The valid modes that the Arduino can be in 
static const uint8_t MODE_IDLE  = 0x00;
static const uint8_t MODE_RC    = 0x01;
static const uint8_t MODE_RPI   = 0x02;
static const uint8_t MODE_DEBUG = 0x03;

// The "registers" in the Arduino that the I2C master can w/r 
static const uint8_t REG_ID        = 0x00;
static const uint8_t REG_MODE      = 0x01;
static const uint8_t REG_STEER     = 0x02;
static const uint8_t REG_SPEED     = 0x03;
static const uint8_t REG_NEXT_READ = 0x04;


// Mode that the Arduino is currently in
static uint8_t mode = MODE_IDLE;

// Left, center, and right on count out of 4095 for steerCNT
static const uint16_t STEER_CNT_MAXLEFT = 570; 
static const uint16_t STEER_CNT_NEUTRAL = 447; 
static const uint16_t STEER_CNT_MAXRIGHT = 350; 

// Forward, stop, reverse on count out of 4095 for motorCNT
static const uint16_t MOTOR_CNT_MAXFOR = 750; // FIXME approx
static const uint16_t MOTOR_CNT_NEUTRAL = 445;
static const uint16_t MOTOR_CNT_MAXREV = 220; // FIXME approx

// On count out of 4095 for the steering / motor PWM signals
static uint16_t steerCNT = STEER_CNT_NEUTRAL;
static uint16_t motorCNT = MOTOR_CNT_NEUTRAL;

// Pulse with in us of the steering / motor PWM signals
uint32_t steerPW = 0; // us
uint32_t motorPW = 0; // us

// Max and min high pulse widths for the steering and motor PWM signals.
// Units are microseconds
static const unsigned int STEER_PW_MIN = 1140; // full right
static const unsigned int STEER_PW_MAX = 2720; // full left
static const unsigned int MOTOR_PW_MIN = 1304; // full finger extend
static const unsigned int MOTOR_PW_MAX = 2650; // full finger contract (trigger pull)

// Frequency of the steerning and motor PWM signals
static const float PWM_FREQ = 72.0; // Hz

// The steering and motor PWM channels on the pwm module
static const uint8_t STEER_CHANNEL = 0;
static const uint8_t MOTOR_CHANNEL = 1;

// PWM module
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_I2C_ADDR);

// LED blinking stuff
static const unsigned char PIN_LED = 13; 
static int LED_state = LOW;
static unsigned long prevMili = 0;
static unsigned long interval = 500;

// Command stuff
const uint8_t cmdBufLen = 128;
char cmdBuf[cmdBufLen];


/* Arduino setup function. Runs on device power on before the loop function is
 * called.
 */ 
void setup()
{
    // Setup serial
    Serial.begin(9600); 

    // Setup GPIO pins
    pinMode(PIN_PWM_IN_S, INPUT);
    pinMode(PIN_PWM_IN_M, INPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LED_state);

    // Initialize mode
    mode = MODE_IDLE;
    
    // Setup pwm module
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);

    // DEBUGGING
    Serial.print("Exiting setup()\n");

    return;
}

/* Arduino loop function. This is the code that runs continuously on the micro-
 * processor.
 */
void loop()
{
    // Get debug commands from serial
    if (mode == MODE_DEBUG) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            Serial.print("c = "); Serial.println(c);
            if (c == 'I') {
                mode = MODE_IDLE;
                Serial.print("mode = "); Serial.println(mode);
            } else if (c == 'R') {
                mode = MODE_RC;
                Serial.print("mode = "); Serial.println(mode);
            } else if (c== 'P') {
                mode = MODE_RPI;
                Serial.print("mode = "); Serial.println(mode);
            } else if (mode == MODE_RPI) {
                if (c == 'l') {
                    steerCNT += 10;   
                    Serial.print("steerCNT = "); Serial.println(steerCNT);
                } else if (c == 'r') {
                    steerCNT -= 10;   
                    Serial.print("steerCNT = "); Serial.println(steerCNT);
                } else if (c == 'f') {
                    motorCNT += 10;   
                    Serial.print("motorCNT = "); Serial.println(motorCNT);
                } else if (c == 'b') {
                    motorCNT -= 10;   
                    Serial.print("motorCNT = "); Serial.println(motorCNT);
                }
            }
            Serial.println("done available");
        }
    }

    // Take in command from serial FIXME
    if ( (Serial.available() > 0) && (mode != MODE_DEBUG) ) {
        Serial.readBytesUntil('\0', cmdBuf, cmdBufLen);
        char * segment = strtok(cmdBuf, " ");
        while (segment != NULL) {
            Serial.println(segment); 
            segment = strtok(NULL, " ");
        }
        Serial.println(cmdBuf); // FIXME print() not println
        Serial.println("------------");
    }

    // Get on count values based on mode
    if (mode == MODE_RC) {
        // Read pulse width of the steering and motor signals
        steerPW = pulseIn(PIN_PWM_IN_S, HIGH); // microseconds
        motorPW = pulseIn(PIN_PWM_IN_M, HIGH); // microseconds

        // DEBUGGING
        //Serial.print("steerPW = "); Serial.println(steerPW);
        //Serial.print("motorPW = "); Serial.println(motorPW);
            
        // Convert to on count
        steerCNT = map(steerPW, STEER_PW_MIN, STEER_PW_MAX, 
                       STEER_CNT_MAXRIGHT, STEER_CNT_MAXLEFT);
        motorCNT = map(motorPW, MOTOR_PW_MIN, MOTOR_PW_MAX,
                       MOTOR_CNT_MAXREV, MOTOR_CNT_MAXFOR); 

        // DEBUGGING
        //Serial.print("steerCNT = "); Serial.println(steerCNT);
        //Serial.print("motorCNT = "); Serial.println(motorCNT);
    } else if (mode == MODE_IDLE) {
        // Make sure steering and motor servos are not doing anything
        steerCNT = STEER_CNT_NEUTRAL; 
        motorCNT = MOTOR_CNT_NEUTRAL;
    } 

    // Write these values to the motor controller and steering servo
    pwm.setPin(STEER_CHANNEL, steerCNT, false);
    pwm.setPin(MOTOR_CHANNEL, motorCNT, false);

    // Blink led to indicate that the board is working
    unsigned long curMili = millis();
    if ( (curMili - prevMili) > interval) { // blink led
        if (LED_state == LOW) {
            LED_state = HIGH;
        } else {
            LED_state = LOW;
        }
        prevMili = curMili;
        digitalWrite(PIN_LED, LED_state); 
    } else if (prevMili > curMili) { // overflow occured
        prevMili = 0;
    } 

} // loop

