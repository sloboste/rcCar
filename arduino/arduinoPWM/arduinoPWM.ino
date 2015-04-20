/* Arduino PWM 
 * Microcomputer-Controlled Car Project
 * University of Michigan - Tilbury Research Group
 * Author: Steven Sloboda
 * Version: 1.0
 */


#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// 7 bit I2C address of PWM module (0x40 is default)
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
const uint16_t MOTOR_CNT_MAXFOR = 750; // FIXME approx
const uint16_t MOTOR_CNT_NEUTRAL = 440;
const uint16_t MOTOR_CNT_MAXREV = 220; // FIXME approx

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
const uint8_t cmdBufLen = 8;
char cmdBuf[cmdBufLen];
struct cmdStruct {
    char func;
    char modeOrDir;
    uint8_t percent;
    bool valid;
};
struct cmdStruct cmd;
const uint8_t respBufLen = 8;
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

    // DEBUGGING
    Serial.print("Exiting setup()\n");

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
    if (Serial.available() > 0) {
        memset(cmdBuf, '\0', cmdBufLen);
        Serial.readBytesUntil('\0', cmdBuf, cmdBufLen);
        //Serial.println(cmdBuf);
        char * segment = strtok(cmdBuf, " \0");
        for (uint8_t i = 0; segment != NULL; ++i) {
            //Serial.print("segment = "); Serial.println(segment);
            if (i == 0) { // func
                memcpy(&cmd.func, segment, 1); 
            } else if (i == 1) { // mode, dir, or null
                if (segment[0] != NULL) { // dir
                    memcpy(&cmd.modeOrDir, segment, 1);
                }
            } else if (i == 2) { // percent or null
                char p[4] = "\0\0\0";
                if (segment[1] == '\0') { 
                    memcpy(p, segment, 1);
                    cmd.percent = atoi(p) ;
                } else if (segment[2] == '\0') {
                     memcpy(p, segment, 2);
                     cmd.percent = atoi(p);
                } else if (segment[3] == '\0') {
                     memcpy(p, segment, 3);
                     cmd.percent = atoi(p);
                }
            } else {
                //Serial.println("ERROR: malformed request");
            }
            segment = strtok(NULL, " \0");
        }
        cmd.valid = true;
        /*
        Serial.print("func = "); Serial.println(cmd.func);
        Serial.print("modeOrDir = "); Serial.println(cmd.modeOrDir);
        Serial.print("percent = "); Serial.println(cmd.percent);
        Serial.print("valid = "); Serial.println(cmd.valid);
        Serial.println("------------");
        */
    }
     
    // Execute command 
    if (!cmd.valid) cmd.func = 'X';
    switch (cmd.func) {
    case 'A': // set mode
        if (cmd.modeOrDir == 'P') mode = MODE_RPI; 
        else if (cmd.modeOrDir == 'R') mode = MODE_RC; 
        else if (cmd.modeOrDir == 'I') mode = MODE_IDLE; 
        // FIXME response
        //Serial.print("A\0");
        break;
    case 'B': // set steer
        // Convert to on count
        if (cmd.modeOrDir == 'L') {
            steerCNT = map(cmd.percent, 0, 100, 
                           STEER_CNT_NEUTRAL, STEER_CNT_MAXLEFT);
        } else if (cmd.modeOrDir == 'R') {
            steerCNT = map(cmd.percent, 0, 100, 
                           STEER_CNT_NEUTRAL, STEER_CNT_MAXRIGHT);
        }
        // FIXME response
        //Serial.print("B\0");
        break;
    case 'C': // set motor
        if (cmd.modeOrDir == 'F') {
            motorCNT = map(cmd.percent, 0, 100, 
                           MOTOR_CNT_NEUTRAL, MOTOR_CNT_MAXFOR);
        } else if (cmd.modeOrDir == 'B') {
            motorCNT = map(cmd.percent, 0, 100, 
                           MOTOR_CNT_NEUTRAL, MOTOR_CNT_MAXREV);
        }
        // FIXME response
        //Serial.print("A\0");
        break;
    case 'D': // get steer
        // FIXME response
        respBuf[0] = cmd.func;
        respBuf[1] = ' ';
        if (steerCNT > STEER_CNT_NEUTRAL) {
            respBuf[2] = 'L';
            itoa(map(steerCNT, STEER_CNT_NEUTRAL, STEER_CNT_MAXLEFT, 0, 100),
                 respBuf, 4
            );
        } else {
            respBuf[2] = 'R';
            itoa(map(steerCNT, STEER_CNT_NEUTRAL, STEER_CNT_MAXRIGHT, 0, 100),
                 respBuf, 4
            );
        }
        respBuf[3] = ' '; 
        respBuf[7] = '\0'; 
        Serial.print(respBuf); 
        break;
    case 'E': // get motor
        // FIXME response
        respBuf[0] = cmd.func;
        respBuf[1] = ' ';
        if (motorCNT > MOTOR_CNT_NEUTRAL) {
            respBuf[2] = 'F';
            itoa(map(motorCNT, MOTOR_CNT_NEUTRAL, MOTOR_CNT_MAXFOR, 0, 100),
                 respBuf, 4
            );
        } else {
            respBuf[2] = 'B';
            itoa(map(motorCNT, MOTOR_CNT_NEUTRAL, MOTOR_CNT_MAXREV, 0, 100),
                 respBuf, 4
            );
        }
        respBuf[3] = ' '; 
        respBuf[7] = '\0'; 
        Serial.print(respBuf);
        break;
    default:
        // Do nothing
        // FIXME response
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

