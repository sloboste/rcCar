/* Arduino PWM 
 * Microcomputer-Controlled Car Project
 * University of Michigan - Tilbury Research Group
 * Author: Steven Sloboda
 * Version: 0.2
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>


// 7-bit I2C address of Arduino (arbitrarily assigned)
const unsigned char SELF_I2C_ADDR = 0x10;

// 8-bit identification number (arbitrarily assigned) of Arduino
const unsigned char SELF_CHIP_ID = 0xAD; 

// Pins used on  the Arduino
const unsigned char PIN_I2C_SDA   = A4; //FIXME analog pin A4
const unsigned char PIN_I2C_SCL   = A5; //FIXME analog pin A5
const unsigned char PIN_PWM_IN_S  = 0; //FIXME digital pin 3, 4, 5, 6, 9, or 10
const unsigned char PIN_PWM_IN_M  = 0; //FIXME ""
const unsigned char PIN_PWM_OUT_S = 0; //FIXME ""
const unsigned char PIN_PWM_OUT_M = 0; //FIXME ""
const unsigned char PIN_LED       = 14; //FIXME digital pin 14

// The valid modes that the Arduino can be in 
const unsigned char MODE_IDLE  = 0x00;
const unsigned char MODE_RC    = 0x01;
const unsigned char MODE_RPI   = 0x02;

// The "registers" in the Arduino that the I2C master can w/r 
const unsigned char REG_ID        = 0x00;
const unsigned char REG_MODE      = 0x01;
const unsigned char REG_STEER     = 0x02;
const unsigned char REG_SPEED     = 0x03;
const unsigned char REG_NEXT_READ = 0x04;

// Mode that the Arduino is currently in
unsigned char mode;

// The register that the Arduino will send contents of on next master read
// Note REG_NEXT_READ is used to represent "invalid" i.e., not supposed to send
unsigned char nextRead = REG_NEXT_READ; 

// Duty cycles for steering and motor PWM signals
unsigned char steerDC = 0;
unsigned char motorDC = 0;

// Max and min high pulse widths for the steering and motor PWM signals
// Out of 4096 i think...
// FIXME
const unsigned int STEER_PW_MIN = 0;
const unsigned int STEER_PW_MAX = 0;
const unsigned int MOTOR_PW_MIN = 0;
const unsigned int MOTOR_PW_MAX = 0;

// Center position for steering and 0% throttle
const unsigned char STEER_DC_NEUTRAL = 50; // FIXME 
const unsigned char MOTOR_DC_NEUTRAL = 50; // FIXME

// Frequency of the steerning and motor PWM signals
const unsigned char PWM_FREQ = 72; // Hz

// The steering and motor PWM channels on the pwm module
const unsigned char STEER_CHANNEL = 0;
const unsigned char MOTOR_CHANNEL = 1;


/* Arduino setup function. Runs on device power on before the loop function is
 * called.
 */ 
void setup()
{
    // Initialize mode
    mode = IDLE;

    // Set i2c bus address
    Wire.begin(SELF_I2C_ADDR); 
    
    // Register handlers for master read and write
    Wire.onRecieve(masterWriteHandler);
    Wire.onRequest(masterReadHandler);

    // Start up the pwm module
    pwm.begin();

    // Set the adafruit pwm module to the correct frequency // FIXME
    setPWMFreq(PWM_FREQ);

    return;
}

/* Arduino loop function. This is the code that runs continuously on the micro-
 * processor.
 */
void loop()
{
    // Take appropriate action based on mode
    switch (mode) {

    case RC:
        // Intercept PWM duty cycles from radio reciever
        interceptPWM(steerDC, motorDC);
            
        // Write these values to the motor controller and steering servo
        writeToSteerServo(steerDC);
        writeToMotorController(motorDC);
        
        break;
                
    case RPI:
        // steerDC will have been set by RPI command
        // motorDC will have been set by RPI command

        // Write these values to the motor controller and steering servo
        writeToSteerServo(steerDC);
        writeToMotorController(motorDC);
        
        break;

    default: // (mode == IDLE) || (mode == some_undefined_value) 
        // Make sure steering and motor servos are not doing anything
        steerDC = STEER_DC_NEUTRAL; 
        motorDC = MOTOR_DC_NEUTRAL;

        // Write these values to the motor controller and steering servo
        writeToSteerServo(steerDC);
        writeToMotorController(motorDC);

        break;
        
    } // switch

} // loop


/* Public: Read the PWM signals that are being sent from the radio transmitter
 *         to the car's radio receiver.
 *
 * Note: The steering servo operates on ?.?V logic with a frequency of ??.???Hz
 *       and a duty cycle between ?? and ?? percent.
 *
 *       The motor controller operates on ?.?V logic with a frequency of 
 *       ??.???Hz and a duty cycle between ?? and ?? percent.
 *
 * steering - Will hold the duty cycle of the PWM signal intended for the
 *            steering servo when the function returns.
 *
 * motor - Will hold the duty cycle of the PWM signal intended for the motor
 *         controller when the function returns. 
 */
void interceptPWM(unsigned char &steering, unsigned char &motor) // FIXME
{
    // Read pulse width of the steering and motor signals
    unsigned int s = pulseIn(PIN_PWM_IN_S, HIGH);
    unsigned int m = pulseIn(PIN_PWM_IN_M, HIGH);

    // Convert the pulse width of the high pulse to a percentage 
    steering = (unsigned char) map(s, STEER_PW_IN_MIN, STEER_PW_IN_MAX, 0, 100);
    steering = (unsigned char) map(m, MOTOR_PW_IN_MIN, MOTOR_PW_IN_MAX, 0, 100);
    
    return;
} // interceptPWM


/* Public: Change the PWM signal that is being sent to the steering servo to
 *         the given duty cycle.
 *
 * Note: The steering servo operates on ?.?V logic with a frequency of ??.???Hz
 *       and a duty cycle between ?? and ?? percent.
 *
 * dutyCycle - The duty cycle of the PWM signal that the steering servo will
 *            recieve after the call to this function.
 */
void writeToSteerServo(unsigned char dutyCycle) // FIXME
{
    // Convert duty cycle to on/off count
    unsigned short on = 0; // Signal starts on by default
    unsigned short off = map(dutycycle, 0, 100, 0, 4095);

    // Tell the pwm module to generate the waveform
    setPWM(STEER_CHANNEL);

    return;
} // writeToSteerServo


/* Public: Change the PWM signal that is being sent to the motor controller to
 *         the given duty cycle.
 *
 * Note: The motor controller operates on ?.?V logic with a frequency of 
 *       ??.???Hz and a duty cycle between ?? and ?? percent.
 *
 * dutyCycle - The duty cycle of the PWM signal that the steering servo will
 *            recieve after the call to this function.
 */
void writeToMotorController(unsigned char dutyCycle) // FIXME
{
    // Convert duty cycle to on/off count
    unsigned short on = 0; // Signal starts on by default
    unsigned short off = map(dutycycle, 0, 100, 0, 4095);

    // Tell the pwm module to generate the waveform
    setPWM(MOTOR_CHANNEL);

    return;
} //writeToMotorController


/* Private: Called when the i2c master reads from the Arduino. A master read is
 *          the same as receiving a transmission in Arduino lingo.
 */
void masterReadHandler()
{
    // Sends data to the master according to the value of nextRead 
    switch (nextRead) {
    case REG_ID:
        // Send the chip id
        Wire.write(SELF_CHIP_ID); 

        break;

    case REG_STEER:
        // Send the steering servo duty cycle
        Wire.write(steerDC); 
        
        break;
    
    case REG_SPEED:
        // Send the motor controller duty cycle
        Wire.write(motorDC); 

        break;
        
    default: // Can't read from this reg
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
    // FIXME in the example code the last byte is ignored should I do that too?
    if (Wire.available() <= 3) { 
        // Determine the selected "register"
        unsigned char reg = Wire.read();
        switch (reg) {
        case REG_MODE:
            // read in the mode that the I2C master wants to put the arduino in
            if (Wire.available() == 1) {
                unsigned char m = Wire.read();
                if ( (m == MODE_IDLE) || (m == MODE_RC) || (m == MODE_RPI) ) {
                    mode = m;
                }
            }

            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid

            break;

        case REG_STEER:
            if (Wire.available() == 2) {
                unsigned char direction = Wire.read();
                unsigned char percent = Wire.read();
                
                // FIXME convert percentage and direction to duty cycle
                if (direction == /*FIXME*/ 0) { // forward
                    speedDC = (unsigned char) map(percent, 0, 100, /*FIXME*/); 
                } else { // reverse
                    speedDC = (unsigned char) map(percent, 0, 100, /*FIXME*/);
                }

            }

            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid

            break;

        case REG_SPEED:
            // Read in the percentage and direction that the I2C master wants
            // the rcCar to turn in
            if (Wire.available() == 2) {
                unsigned char direction = Wire.read();
                unsigned char percent = Wire.read();
                
                // FIXME convert percentage and direction to duty cycle
                if (direction == /*FIXME*/ 0) { // left
                    speedDC = (unsigned char) map(percent, 0, 100, /*FIXME*/); 
                } else { // right
                    speedDC = (unsigned char) map(percent, 0, 100, /*FIXME*/);
                }

            }

            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid

            break;

        case REG_NEXT_READ:
            // Read in the number of the next reg to read
            if (Wire.available() == 1) {
                unsigned char r = Wire.read();
                if ( (r == REG_ID) || (r == REG_STEER) || (r == REG_SPEED) ) {
                    nextRead = r;
                } else {
                    nextRead = REG_NEXT_READ; // invalid
                }
            }

            break;

        default: // Bad reg name
            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid

            break;

        } // switch

    } // if

} // masterWriteHandler

