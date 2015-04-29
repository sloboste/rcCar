/* ArduinoPWM.ino
 * Program to control RC car with an Arduino 
 * Microcomputer-Controlled Car Project
 * University of Michigan - Tilbury Research Group
 * Version: 1.0
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// 7 bit I2C address of PWM module (0x40 is default)
// Note: changing this requires soldering the board
const uint8_t PWM_I2C_ADDR = 0x40; 

// Pins used on  the Arduino
//const unsigned char PIN_I2C_SDA   = A4; // SDA 
//const unsigned char PIN_I2C_SCL   = A5; // SCL
const uint8_t PIN_PWM_IN_S  = 3; // Receiver channel 1 
const uint8_t PIN_PWM_IN_M  = 5; // Receiver channel 2
// Data line close to the red button on the receiver...

// The valid modes that the Arduino can be in 
const uint8_t MODE_IDLE  = 0x00;
const uint8_t MODE_RC    = 0x01;
const uint8_t MODE_RPI   = 0x02;

// Mode that the Arduino is currently in
uint8_t mode = MODE_IDLE;

// Left, center, and right on count out of 4095 for steerCNT
const uint16_t STEER_CNT_MAXLEFT = 570; 
const uint16_t STEER_CNT_NEUTRAL = 447; 
const uint16_t STEER_CNT_MAXRIGHT = 350; 

// Forward, stop, reverse on count out of 4095 for motorCNT
const uint16_t MOTOR_CNT_MAXFOR = 750; 
const uint16_t MOTOR_CNT_NEUTRAL = 390;
const uint16_t MOTOR_CNT_MAXREV = 220; 

// On count out of 4095 for the steering / motor PWM signals
uint16_t steerCNT = STEER_CNT_NEUTRAL;
uint16_t motorCNT = MOTOR_CNT_NEUTRAL;

// Pulse with in us of the steering / motor PWM signals
uint32_t steerPW = 0; // us
uint32_t motorPW = 0; // us

// Max and min high pulse widths for the steering and motor PWM signals.
// Units are microseconds
const unsigned int STEER_PW_MIN = 1140; // full right
const unsigned int STEER_PW_MAX = 2720; // full left
const unsigned int MOTOR_PW_MIN = 1304; // full finger extend
const unsigned int MOTOR_PW_MAX = 2650; // full finger contract (trigger pull)

// Frequency of the steerning and motor PWM signals
const float PWM_FREQ = 72.0; // Hz

// The steering and motor PWM channels on the pwm module
const uint8_t STEER_CHANNEL = 0;
const uint8_t MOTOR_CHANNEL = 1;

// PWM module
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_I2C_ADDR);

// LED blinking
const uint8_t PIN_LED = 13; 
int LED_state = LOW;
unsigned long prevMili = 0;
unsigned long interval = 500;

// Serial command structure
const uint8_t cmdBufLen = 6;
char cmdBuf[cmdBufLen];
struct cmdStruct {
    char func;
    char mode;
    int8_t percent;
    bool valid;
};
struct cmdStruct cmd;

// Serial response
const uint8_t respBufLen = 13;
char respBuf[respBufLen];

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

    // Init cmd
    cmd.valid = false;

    return;
}

/* Arduino loop function. This is the code that runs continuously on the micro-
 * processor.
 */
void loop()
{
    // Get RPI command from serial
    getCmd();
    //getCmdDebug();

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

    // Let us see if the board is running the code
    blinkLED();

} // loop

void getCmd()
{
    // Take in command from serial
    cmd.valid = false;
    if (Serial.available()) {
        memset(cmdBuf, '\0', cmdBufLen);
        Serial.readBytesUntil('\0', cmdBuf, cmdBufLen);
        //Serial.println(cmdBuf);
        char * segment = strtok(cmdBuf, " \0");
        for (uint8_t i = 0; segment != NULL; ++i) {
            //Serial.print("i = "); Serial.print(i);
            //Serial.print("; segment = "); Serial.println(segment);
            if (i == 0) { // func
                memcpy(&cmd.func, segment, 1);
            } else if ( (i == 1) && (cmd.func == 'A') ) { // mode
                memcpy(&cmd.mode, segment, 1);
            } else { // percent
                //Serial.print("num digits = "); Serial.println(strlen(segment));
                cmd.percent = atoi(segment);
            }
            segment = strtok(NULL, " \0");
        }
        cmd.valid = true;
        /* 
        Serial.print("func = "); Serial.println(cmd.func);
        Serial.print("mode = "); Serial.println(cmd.mode);
        Serial.print("percent = "); Serial.println(cmd.percent);
        Serial.print("valid = "); Serial.println(cmd.valid);
        Serial.println("------------");
        */ 
    }
     
    // Execute command 
    uint8_t len;
    char num[5]; 
    if (!cmd.valid) cmd.func = 'X';
    switch (cmd.func) {
    case 'A': // set mode
        if (cmd.mode == 'P') mode = MODE_RPI; 
        else if (cmd.mode == 'R') mode = MODE_RC; 
        else if (cmd.mode == 'I') mode = MODE_IDLE; 
        break;
    case 'B': // set steer
        if (mode != MODE_RPI) break;
        // Convert to on count
        if (cmd.percent < 0) {
            steerCNT = map(cmd.percent, -100, 0, 
                           STEER_CNT_MAXLEFT, STEER_CNT_NEUTRAL);
        } else {
            steerCNT = map(cmd.percent, 0, 100, 
                           STEER_CNT_NEUTRAL, STEER_CNT_MAXRIGHT);
        }
        break;
    case 'C': // set motor throttle
        if (mode != MODE_RPI) break;
        if (cmd.percent > 0) {
            motorCNT = map(cmd.percent, 0, 100, 
                           MOTOR_CNT_NEUTRAL, MOTOR_CNT_MAXFOR);
        } else {
            motorCNT = map(cmd.percent, -100, 0, 
                           MOTOR_CNT_MAXREV, MOTOR_CNT_NEUTRAL);
        }
        break;
    case 'D': // get steering and motor throttle data  
        // respBuf char[13]; char num[5]; 
        // Func
        respBuf[0] = 'E';
        // Space
        respBuf[1] = ' ';
        len = 2;
        // Steering
        memset(num, '\0', 5);
        if (steerCNT > STEER_CNT_NEUTRAL) {
            itoa(map(steerCNT, STEER_CNT_MAXLEFT, STEER_CNT_NEUTRAL, -100, 0),
                 num, 10);
        } else {
            itoa(map(steerCNT, STEER_CNT_NEUTRAL, STEER_CNT_MAXRIGHT, 0, 100),
                 num, 10);
        }
        memcpy(respBuf+len, num, strlen(num));
        len += strlen(num);
        // Space
        respBuf[len++] = ' '; 
        // Motor throttle
        memset(num, '\0', 5);
        if (motorCNT > MOTOR_CNT_NEUTRAL) {
            itoa(map(motorCNT, MOTOR_CNT_NEUTRAL, MOTOR_CNT_MAXFOR, 0, 100),
                 num, 10);
        } else {
            itoa(map(motorCNT, MOTOR_CNT_MAXREV, MOTOR_CNT_NEUTRAL, -100, 0),
                 num, 10);
        }
        memcpy(respBuf+len, num, strlen(num));
        len += strlen(num);
        // Newline
        respBuf[len++] = '\n'; 
        // Null terminate
        respBuf[len++] = '\0'; 
        // Send response
        Serial.write((uint8_t *) respBuf, len);
        break;
    default:
        // Do nothing
        break;
    }

    return;
}

void getCmdDebug()
{
    // Get debug commands from serial
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

void blinkLED()
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
        digitalWrite(PIN_LED, LED_state); 
    } else if (prevMili > curMili) { // overflow occured
        prevMili = 0;
    } 
}

