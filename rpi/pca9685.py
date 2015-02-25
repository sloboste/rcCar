
import smbus

class PCA9685:
    def _init(self):
        # Set up i2c bus on RPi
        self.bus = smbus.SMBus(1) # /dev/i2c-1 
        # Characteristics
        self.OSC_CLK_FREQ      = 25000000
        self.I2C_ADDR_7BIT     = 0x00 # FIXME could be 0x00 or 0xE0 (pg 7)
        # Programmable I2C addresses (NOT sub-addresses!)
        self.SOFT_RESET        = 0x06 # (pg 6)
        self.LED_ALL_CALL      = 0xE0
        # Internal regs that we can write to
        self.REG_MODE1         = 0x00
        self.REG_MODE2         = 0x01
        self.REG_SUBADR1       = 0x02
        self.REG_SUBADR2       = 0x03
        self.REG_SUBADR3       = 0x04
        self.REG_ALLCALLARD    = 0x05
        self.REG_LED0_ON_L     = 0x06
        self.REG_LED0_ON_H     = 0x07
        self.REG_LED0_OFF_L    = 0x08
        self.REG_LED0_OFF_H    = 0x09
        self.REG_LED1_ON_L     = 0x0A
        self.REG_LED1_ON_H     = 0x0B
        self.REG_LED1_OFF_L    = 0x0C
        self.REG_LED1_OFF_H    = 0x0D
        self.REG_LED2_ON_L     = 0x0E
        self.REG_LED2_ON_H     = 0x0F
        self.REG_LED2_OFF_L    = 0x10
        self.REG_LED2_OFF_H    = 0x11
        # continue through 69 (base 10)
        self.REG_ALL_LED_ON_L  = 0xFA
        self.REG_ALL_LED_ON_H  = 0xFB
        self.REG_ALL_LED_OFF_L = 0xFC
        self.REG_ALL_LED_OFF_H = 0xFD
        self.REG_PRESCALE      = 0xFE
        self.REG_TEST_MODE     = 0xFF
        # 
        
    def setFandDC(self, frequency):
        # Validate frequency
        if ((frequency < 40) || (frequency > 1000)):
            raise ValueError("output pin must be between 0 and 1000 inclusive")
        # Determine prescaler for frequency
        prescaler = round( self.OSC_CLOCK / (4096 * frequency) )
        # Sleep device to allow frequency change

        # Set prescaler value
        self.bus.write_byte_data(self.I2C_ADDR_7BIT, self.REG_PRESCALE, 
        # Wake up device


    def setDC(self, outputPin, dutyCycle):
        # Validate output pin
        if ((outputPin < 0) || (outputPin > 15)):
            raise ValueError("output pin must be between 0 and 15 inclusive")
        # Validate dutyCycle
        if ((dutyCycle < 0) || (dutyCycle > 100)):
            raise ValueError("output pin must be between 0 and 100 inclusive")
        # Determine on/off times for dutyCycle: note inverted output
        # onCount = 0   
        offCount = round( (1-(dutyCycle/100)) * 4096 )
        offL = 0x0011 & offCount
        offH = (0x1100 & offcount) >> 4
        # Determine base reg address for output pin
        baseAddr = outputPin + self.REG_LED0_ON_L
        # Write to device: ON_L, ON_H, OFF_L, OFF_H
        self.bus.write_byte_data(self.I2C_ADDR_7BIT, baseAddr, 0) 
        self.bus.write_byte_data(self.I2C_ADDR_7BIT, baseAddr+1, 0)
        self.bus.write_byte_data(self.I2C_ADDR_7BIT, baseAddr+2, offL)
        self.bus.write_byte_data(self.I2C_ADDR_7BIT, baseAddr+3, offH)

