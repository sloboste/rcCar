/* Arduino PWM 
 * Microcomputer-Controlled Car Project
 * University of Michigan - Tilbury Research Group
 * Author: Steven Sloboda
 * Version: 0.3
 */

// Comment out this line for production version of the code
#define DEBUG // FIXME

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

// The "registers" in the Arduino that the I2C master can w/r 
static const uint8_t REG_ID        = 0x00;
static const uint8_t REG_MODE      = 0x01;
static const uint8_t REG_STEER     = 0x02;
static const uint8_t REG_SPEED     = 0x03;
static const uint8_t REG_NEXT_READ = 0x04;


// Mode that the Arduino is currently in
static uint8_t mode = MODE_IDLE;

// The register that the Arduino will send contents of on next master read
// Note REG_NEXT_READ is used to represent "invalid" i.e., not supposed to send
static uint8_t nextRead = REG_NEXT_READ; // FIXME not used 

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

// DEBUGGING
#ifdef DEBUG
// LED blinking stuff
static const unsigned char PIN_LED = 13; 
static int LED_state = LOW;
static unsigned long prevMili = 0;
static unsigned long interval = 500;
#endif


/* Arduino setup function. Runs on device power on before the loop function is
 * called.
 */ 
void setup()
{
    // DEBUGGING
    #ifdef DEBUG
    Serial.begin(9600); 
    Serial.print("Entered setup()\n");
    #endif

    // Setup GPIO pins
    pinMode(PIN_PWM_IN_S, INPUT);
    pinMode(PIN_PWM_IN_M, INPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LED_state);

    // Initialize mode
    mode = MODE_IDLE;

    // Start up the pwm module
    pwm.begin();

    // Set the adafruit pwm module to the correct frequency 
    pwm.setPWMFreq(PWM_FREQ);

    // DEBUGGING
    #ifdef DEBUG
    Serial.print("Exiting setup()\n");
    #endif

    return;
}

/* Arduino loop function. This is the code that runs continuously on the micro-
 * processor.
 */
void loop()
{
    // DEBUGGING
    #ifdef DEBUG
    //Serial.print("Entered loop()\n");
    #endif

    // Take in mode command from serial
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

    // Take appropriate action based on mode
    switch (mode) {

    case MODE_RC:
        // Read pulse width of the steering and motor signals
        steerPW = pulseIn(PIN_PWM_IN_S, HIGH); // microseconds
        motorPW = pulseIn(PIN_PWM_IN_M, HIGH); // microseconds

        // DEBUGGING
        //Serial.print("steerPW = "); Serial.println(steerPW);
        Serial.print("motorPW = "); Serial.println(motorPW);
            
        // Convert to on count
        steerCNT = map(steerPW, STEER_PW_MIN, STEER_PW_MAX, 
                       STEER_CNT_MAXRIGHT, STEER_CNT_MAXLEFT);
        motorCNT = map(motorPW, MOTOR_PW_MIN, MOTOR_PW_MAX,
                       MOTOR_CNT_MAXREV, MOTOR_CNT_MAXFOR); 

        // DEBUGGING
        //Serial.print("steerCNT = "); Serial.println(steerCNT);
        //Serial.print("motorCNT = "); Serial.println(motorCNT);

        // Write these values to the motor controller and steering servo
        pwm.setPin(STEER_CHANNEL, steerCNT, false);
        pwm.setPin(MOTOR_CHANNEL, motorCNT, false);

        break;
                
    case MODE_RPI:
        // On counts will be set by this point

        // Write these values to the motor controller and steering servo
        pwm.setPin(STEER_CHANNEL, steerCNT, false);
        pwm.setPin(MOTOR_CHANNEL, motorCNT, false);
        
        break;

    default: // (mode == MODE_IDLE) || (mode == some_undefined_value) 
        // Make sure steering and motor servos are not doing anything
        steerCNT = STEER_CNT_NEUTRAL; 
        motorCNT = MOTOR_CNT_NEUTRAL;

        // Write these values to the motor controller and steering servo
        pwm.setPin(STEER_CHANNEL, steerCNT, false);
        pwm.setPin(MOTOR_CHANNEL, motorCNT, false);

        break;
        
    } // switch

    // DEBUGGING
    #ifdef DEBUG
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
    //Serial.print("Exiting loop()\n");
    //delay(1000);
    #endif

} // loop








/* Private: Called when the i2c master reads from the Arduino. A master read is
 *          the same as receiving a transmission in Arduino lingo.
 */
void masterReadHandler()
{
    // DEBUGGING
    Serial.print("Enter masterReadHandler()\n");
   
    // Sends data to the master according to the value of nextRead 
    switch (nextRead) {
    case REG_ID:
        // DEBUGGING
        Serial.print("nextRead was REG_ID\n");
    
        // Send the chip id
        Wire.write(SELF_CHIP_ID); 

        break;

    case REG_STEER:
        // DEBUGGING
        Serial.print("nextRead was REG_STEER\n");
    
        // Send the steering servo duty cycle
        //Wire.write(steerDC); 
        
        break;
    
    case REG_SPEED:
        // DEBUGGING
        Serial.print("nextRead was REG_SPEED\n");
    
        // Send the motor controller duty cycle
        //Wire.write(motorDC); 

        break;
        
    default: // Can't read from this reg
        // DEBUGGING
        Serial.print("next read was nothing we could do\n");
    
        // Do nothing
        break; 
    }

    // Reset nextRead
    nextRead = REG_NEXT_READ; // invalid

    return;
} // masterReadHandler


/* Private: Called when the i2c master writes to the Arduino. A master write is
 *          the same as receiving a request in Arduino lingo.
 * 
 * Note: this function does nothing if the write is malformed
 *
 * numBytes - number of bytes that the master sent to the Arduino slave
 *
 */
void masterWriteHandler(int numBytes)
{
    // DEBUGGING
    Serial.print("Enter masterWriteHandler()\n");
 
    // FIXME in the example code the last byte is ignored should I do that too?
    if (Wire.available() <= 3) { 
        // Determine the selected "register"
        unsigned char reg = Wire.read();
        switch (reg) {
        case REG_MODE:
            // DEBUGGING
            Serial.print("register selected was REG_MODE\n");
    
            // read in the mode that the I2C master wants to put the arduino in
            if (Wire.available() == 1) {
                unsigned char m = Wire.read();
                if ( (m == MODE_IDLE) || (m == MODE_RC) || (m == MODE_RPI) ) {
                    mode = m;
                } else { // Invalid mode
                    // DEBUGGING
                    Serial.print("invalid mode selected\n");
                }
            } else { // No mode specified
                // DEBUGGING
                Serial.print("no mode data recieved\n");
            }

            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid

            break;

        case REG_STEER:
            // DEBUGGING
            Serial.print("register selected was REG_STEER\n");
    
            // read in the reg that the I2C master wants to read from
            if (Wire.available() == 2) {
                unsigned char direction = Wire.read();
                unsigned char percent = Wire.read();
                
                // DEBUGGING
                Serial.print("direction = ");
                Serial.println(direction);
                Serial.print("percentage = ");
                Serial.println(percent);

                // FIXME convert percentage and direction to duty cycle
                if (direction == /*FIXME*/ 0) { // forward

                    //steerDC = (unsigned char) map(percent, 0, 100, /*FIXME*/0,0); 
                } else { // reverse

                    //steerDC = (unsigned char) map(percent, 0, 100, /*FIXME*/0,0);
                }

            } else { // No direction or percentage specified
                // DEBUGGING
                Serial.print("no direction or percentage data recieved\n");
            }

            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid

            break;

        case REG_SPEED:
            // DEBUGGING
            Serial.print("register selected was REG_SPEED\n");
    
            // Read in the percentage and direction that the I2C master wants
            // the rcCar to turn in
            if (Wire.available() == 2) {
                unsigned char direction = Wire.read();
                unsigned char percent = Wire.read();
                
                // DEBUGGING
                Serial.print("direction = ");
                Serial.println(direction);
                Serial.print("percentage = ");
                Serial.println(percent);

                // FIXME convert percentage and direction to duty cycle
                if (direction == /*FIXME*/ 0) { // left

                    //motorDC = (unsigned char) map(percent, 0, 100, /*FIXME*/0,0); 
                } else { // right

                    //motorDC = (unsigned char) map(percent, 0, 100, /*FIXME*/0,0);
                }

            } else { // No direction or percentage specified
                // DEBUGGING
                Serial.print("no direction or percentage data recieved\n");
            }

            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid

            break;

        case REG_NEXT_READ:
            // DEBUGGING
            Serial.print("register selected was REG_NEXT_READ\n");
    
            // Read in the number of the next reg to read
            if (Wire.available() == 1) {
                unsigned char r = Wire.read();
                if ( (r == REG_ID) || (r == REG_STEER) || (r == REG_SPEED) ) {
                    nextRead = r;
                    // DEBUGGING
                    Serial.print("master wants to read ");
                    Serial.print(r);
                    Serial.println();
                } else {
                    // DEBUGGING
                    Serial.print("invalid reg selected\n");

                    nextRead = REG_NEXT_READ; // invalid
                }
            } else { // No next reg specified
                // DEBUGGING
                Serial.print("no rext read reg data recieved\n");
            }

            break;

        default: // Bad reg name
            // DEBUGGING
            Serial.print("invalid reg selected\n");

            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid

            break;

        } // switch

    } else { // Data wrong length
       // DEBUGGING
       Serial.print("data wrong length\n");
    }
} // masterWriteHandler

