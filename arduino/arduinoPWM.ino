/* Arduino PWM 
 * Microcomputer-Controlled Car Project
 * University of Michigan - Tilbury Research Group
 * Author: Steven Sloboda
 * Version: 0.1
 */

// 7-bit I2C address of Arduino
const unsigned char SELF_I2C_ADDR = 0x10; // FIXME

// 8-bit identification number (arbitrarily assigned) of Arduino
const unsigned char SELF_ID = 0xAD; // FIXME

// Defines the valid modes that the Arduino can be in 
enum modeEnum {
    IDLE    = 0x00,
    RC      = 0x01,
    RPI     = 0x02,
    SLEEP   = 0x03
};

// Defines the valid commands that the I2C master can send the Arduino
enum cmdEnum {
    ENTER_IDLE_MODE   = 0x00,
    ENTER_RC_MODE     = 0x01,
    ENTER_RPI_MODE    = 0x02,
    ENTER_SLEEP_MODE  = 0x03,
    RETURN_STEER_DC   = 0x04,
    RETURN_MOTOR_DC   = 0x05,
    SET_STEER_DC      = 0x06,
    SET_MOTOR_DC      = 0x07
};

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
    // NOT YET IMPLEMENTED!!!!!!!!!
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
    // NOT YET IMPLEMENTED!!!!!!!!!
}

/* Private: Called when the i2c master reads from the Arduino. A master read is
 *          the same as receiving a transmission in Arduino lingo.
 * 
 * args - ?????????????
 *
 */
void masterReadHandler(/*args*/)
{
    // NOT YET IMPLEMENTED!!!!!!!!!
    // Call different function based on command received
    switch (cmd) {
    case 0x00:
        break;
    default:
        break; 
    }
}

/* Private: Called when the i2c master writes to the Arduino. A master write is
 *          the same as receiving a request in Arduino lingo.
 * 
 * args - ?????????????
 *
 */
void masterWriteHandler(/*args*/)
{
    // NOT YET IMPLEMENTED!!!!!!!!!
    // Call different function based on command received
    switch (cmd) {
    case 0x00:
        break;
    default:
        break; 
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
    // NOT YET IMPLEMENTED!!!!!!!!!
}


/* Interrupt driven input that allows the I2C master to wake the Arduino from 
 * sleep mode
 */
volatile bool wakeup;

// Mode that the Arduino is currently in
enum modeEnum mode;


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
    switch (mode) {
    case RC:
        // Intercept PWM values from radio reciever

        // Write these values to the motor controller and steering servo

        break;
    case RPI:
        // Write values to motor controller and steering servo based on RPI

        break;
    case SLEEP:
        // Put the arduino to sleep FIXME

        break;
    default: // (mode == IDLE) || (mode == undefined) 
        // Do nothing
        break;
    }
}

