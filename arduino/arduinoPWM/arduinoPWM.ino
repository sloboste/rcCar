#include <Wire.h>

/* Arduino PWM 
 * Microcomputer-Controlled Car Project
 * University of Michigan - Tilbury Research Group
 * Author: Steven Sloboda
 * Version: 0.1
 */

// 7-bit I2C address of Arduino (arbitrarily assigned)
const unsigned char SELF_I2C_ADDR = 0x10;

// 8-bit identification number (arbitrarily assigned) of Arduino
const unsigned char SELF_CHIP_ID = 0xAD; 

// Pins used on  the Arduino
const unsigned char PIN_I2C_SDA   = 0; //FIXME
const unsigned char PIN_I2C_SCL   = 0; //FIXME
const unsigned char PIN_INTRPT    = 0; //FIXME
const unsigned char PIN_PWM_IN_S  = 0; //FIXME
const unsigned char PIN_PWM_IN_M  = 0; //FIXME
const unsigned char PIN_PWM_OUT_S = 0; //FIXME
const unsigned char PIN_PWM_OUT_M = 0; //FIXME
const unsigned char PIN_LED       = 0; //FIXME

// The valid modes that the Arduino can be in 
const unsigned char MODE_IDLE  = 0x00;
const unsigned char MODE_RC    = 0x01;
const unsigned char MODE_RPI   = 0x02;
const unsigned char MODE_SLEEP = 0x03;

// The "registers" in the Arduino that the I2C master can w/r 
const unsigned char REG_ID        = 0x00;
const unsigned char REG_MODE      = 0x01;
const unsigned char REG_STEER     = 0x02;
const unsigned char REG_SPEED     = 0x03;
const unsigned char REG_NEXT_READ = 0x04;

/* Interrupt driven input that allows the I2C master to wake the Arduino from 
 * sleep mode
 */
volatile bool wakeup;

// Mode that the Arduino is currently in
unsigned char mode;

// The register that the Arduino will send contents of on next master read
// Note REG_NEXT_READ is used to represent "invalid" i.e., not supposed to send
unsigned char nextRead = REG_NEXT_READ; 

/*
// Duty cycles for steering and motor PWM signals
unsigned char steer = 0;
unsigned char speed = 0;
*/

/* Arduino setup function. Runs on device power on before the loop function is
 * called.
 */ 
void setup()
{
    // Initialize mode
    mode = IDLE;

    // Initialize wakeup
    wakeup = false;
  
    // Set i2c bus address
    Wire.begin(SELF_I2C_ADDR); 
    
    // Register handlers for master read and write
    Wire.onRecieve(masterWriteHandler);
    Wire.onRequest(masterReadHandler);
}

/* Arduino loop function. This is the code that runs continuously on the micro-
 * processor.
 */
void loop()
{
    /*
    // Check if we have just woken up
    if (wakeup) {
        // Reset wakeup flag
        wakeup = false;

        // Default to IDLE mode after being woken up
        mode = IDLE

        // Do something else FIXME

    }

    // Take appropriate action based on mode
    switch (mode) {
    case RC:
        // Intercept PWM values from radio reciever
        interceptPWM(steer, motor);
        // Write these values to the motor controller and steering servo
        setSteer(steer);
        setSpeed(speed);
        break;
    case RPI:
        // Write values to motor controller and steering servo based on RPI
        // steer and speed get values when commands are received from I2C master
        setSteer(steer);
        setSpeed(speed);
        break;
    case SLEEP:
        // Make sure the servos dont do anything FIXME

        // Put the arduino to sleep // FIXME
        sleep_enable();
        attachInterrupt(PIN_INTRPT, _ISR_wakeupHandler, RISING);
        set_sleep_mode( look up args );
        noInterrupts(); // Same as cli();
        sleep_bod_disable(); // Disable brown out detection to save power
        interrupts(); // Same as sei();
        sleep_cpu();
        // Wakes up here
        break;
    default: // (mode == IDLE) || (mode == undefined) 
        // Do nothing
        // Or maybe blink the led so we know it is idle and not fried
        break;
    }
    */
}

// FUNCTIONS DEFINED BELOW

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
void interceptPWM(unsigned char &steering, unsigned char &motor)
{
    // NOT YET IMPLEMENTED!!!!!!!!!
}

/* Public: Change the PWM signal that is being sent to the steering servo to
 *         the given duty cycle.
 *
 * Note: The steering servo operates on ?.?V logic with a frequency of ??.???Hz
 *       and a duty cycle between ?? and ?? percent.
 *
 * dutyCycle - The duty cycle of the PWM signal that the steering servo will
 *            recieve after the call to this function.
 */
void setSteer(unsigned char dutyCycle)
{
    // FIXME frequency of the signal...
    analogWrite(PIN_PWM_OUT_S);
}

/* Public: Change the PWM signal that is being sent to the motor controller to
 *         the given duty cycle.
 *
 * Note: The motor controller operates on ?.?V logic with a frequency of 
 *       ??.???Hz and a duty cycle between ?? and ?? percent.
 *
 * dutyCycle - The duty cycle of the PWM signal that the steering servo will
 *            recieve after the call to this function.
 */
void setSpeed(unsigned char dutyCycle)
{
    // FIXME frequency of the signal...
    analogWrite(PIN_PWM_OUT_M);
}

/* Private: Called when the i2c master reads from the Arduino. A master read is
 *          the same as receiving a transmission in Arduino lingo.
 */
void masterReadHandler()
{
    // Sends data to the master according to the value of nextRead 
    switch (nextRead) {
    case REG_ID:
        Wire.write(SELF_CHIP_ID); 
        break;
    case REG_STEER:
        //FIXME Wire.write(steer); 
        break;
    case REG_SPEED:
        //FIXME Wire.write(speed); 
        break;
    default: // Can't read from this reg
        // Do nothing
        break; 
    }
    // Reset nextRead
    nextRead = REG_NEXT_READ; // invalid
}

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
            if (Wire.available() == 1) {
                unsigned char m = Wire.read();
                if ( (m == MODE_IDLE) || (m == MODE_RC) || 
                     (m == MODE_RPI) || (m == MODE_SLEEP) ) {
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
                setSteer(/*dutycycle*/);
            }
            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid
            break;
        case REG_SPEED:
            if (Wire.available() == 2) {
                unsigned char direction = Wire.read();
                unsigned char percent = Wire.read();
                // FIXME convert percentage and direction to duty cycle
                setSpeed(/*dutycycle*/);
            }
            // Make sure we don't mess up a read procedure
            nextRead = REG_NEXT_READ; // invalid
            break;
        case REG_NEXT_READ:
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
        }
    }
}

/* Private: Interrupt service routine to handle the interrupt caused by a pin
 *          going high that indicates that the Arduino should resume from sleep
 *          mode. 
 * 
 * args - ?????????????
 *
 */
void _ISR_wakeupHandler(/*args*/)
{
    sleep_disable();
    detachInterrupt(/*look up what the args are*/); // FIXME
    wakeup = true;
}
