
from __future__ import print_function


from smbus import SMBus

_DEFAULT_NAME = "Qwiic Proximity Sensor"

_AVAILABLE_I2C_ADDRESS = [0x60]

LOWER = True
UPPER = False

VCNL4040_ALS_CONF = 0x00
VCNL4040_ALS_THDH = 0x01
VCNL4040_ALS_THDL = 0x02
VCNL4040_PS_CONF1 = 0x03
VCNL4040_PS_CONF2 = 0x03
VCNL4040_PS_CONF3 = 0x04
VCNL4040_PS_MS = 0x04
VCNL4040_PS_CANC = 0x05
VCNL4040_PS_THDL = 0x06
VCNL4040_PS_THDH = 0x07
VCNL4040_PS_DATA = 0x08
VCNL4040_ALS_DATA = 0x09
VCNL4040_WHITE_DATA = 0x0A
VCNL4040_INT_FLAG = 0x0B
VCNL4040_ID = 0x0C

class QwiicProximity0(object):
    """
    QwiicProximity

        :param address: The I2C address to use for the device.
                        If not provided, the default address is used.
        :param i2c_driver: An existing i2c driver object. If not provided
                        a driver object is created.
        :return: The Proximity device object.
        :rtype: Object
    """

    # Constructor
    device_name = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    # Flags exposed in userspace - make these class vars
    VCNL4040_PS_INT_MASK = (~((1 << 1) | (1 << 0))) & 0xff
    VCNL4040_PS_INT_DISABLE = 0
    VCNL4040_PS_INT_CLOSE = (1 << 0)
    VCNL4040_PS_INT_AWAY = (1 << 1)
    VCNL4040_PS_INT_BOTH = (1 << 1) | (1 << 0)

    # Define commands/commandwords for the sensor
    VCNL4040_ALS_IT_MASK = (~((1 << 7) | (1 << 6))) & 0xff #uint8
    VCNL4040_ALS_IT_80MS = 0
    VCNL4040_ALS_IT_160MS = (1 << 7)
    VCNL4040_ALS_IT_320MS = (1 << 6)
    VCNL4040_ALS_IT_640MS = (1 << 7) | (1 << 6)

    VCNL4040_ALS_PERS_MASK = (~((1 << 3) | (1 << 2))) & 0xff
    VCNL4040_ALS_PERS_1 = 0
    VCNL4040_ALS_PERS_2 = (1 << 2)
    VCNL4040_ALS_PERS_4 = (1 << 3)
    VCNL4040_ALS_PERS_8 = (1 << 3) | (1 << 2)

    VCNL4040_ALS_INT_EN_MASK = (~(1 << 1)) & 0xff
    VCNL4040_ALS_INT_DISABLE = 0
    VCNL4040_ALS_INT_ENABLE = (1 << 1)

    VCNL4040_ALS_SD_MASK = (~(1 << 0)) & 0xff
    VCNL4040_ALS_SD_POWER_ON = 0
    VCNL4040_ALS_SD_POWER_OFF = (1 << 0)

    VCNL4040_PS_DUTY_MASK = (~((1 << 7) | (1 << 6))) & 0xff
    VCNL4040_PS_DUTY_40 = 0
    VCNL4040_PS_DUTY_80 = (1 << 6)
    VCNL4040_PS_DUTY_160 = (1 << 7)
    VCNL4040_PS_DUTY_320 = (1 << 7) | (1 << 6)

    VCNL4040_PS_PERS_MASK = (~((1 << 5) | (1 << 4))) & 0xff
    VCNL4040_PS_PERS_1 = 0
    VCNL4040_PS_PERS_2 = (1 << 4)
    VCNL4040_PS_PERS_3 = (1 << 5)
    VCNL4040_PS_PERS_4 = (1 << 5) | (1 << 4)

    VCNL4040_PS_IT_MASK = (~((1 << 3) | (1 << 2) | (1 << 1))) & 0xff
    VCNL4040_PS_IT_1T = 0
    VCNL4040_PS_IT_15T = (1 << 1)
    VCNL4040_PS_IT_2T = (1 << 2)
    VCNL4040_PS_IT_25T = (1 << 2) | (1 << 1)
    VCNL4040_PS_IT_3T = (1 << 3)
    VCNL4040_PS_IT_35T = (1 << 3) | (1 << 1)
    VCNL4040_PS_IT_4T = (1 << 3) | (1 << 2)
    VCNL4040_PS_IT_8T = (1 << 3) | (1 << 2) | (1 << 1)

    VCNL4040_PS_SD_MASK = (~(1 << 0)) & 0xff
    VCNL4040_PS_SD_POWER_ON = 0
    VCNL4040_PS_SD_POWER_OFF = (1 << 0)

    VCNL4040_PS_HD_MASK = (~(1 << 3)) & 0xff
    VCNL4040_PS_HD_12_BIT = 0
    VCNL4040_PS_HD_16_BIT = (1 << 3)

    VCNL4040_PS_SMART_PERS_MASK = (~(1 << 4)) & 0xff
    VCNL4040_PS_SMART_PERS_DISABLE = 0
    VCNL4040_PS_SMART_PERS_ENABLE = (1 << 1)

    VCNL4040_PS_AF_MASK = (~(1 << 3)) & 0xff
    VCNL4040_PS_AF_DISABLE = 0
    VCNL4040_PS_AF_ENABLE = (1 << 3)

    VCNL4040_PS_TRIG_MASK = (~(1 << 3)) & 0xff
    VCNL4040_PS_TRIG_TRIGGER = (1 << 2)

    VCNL4040_WHITE_EN_MASK = (~(1 << 7)) & 0xff
    VCNL4040_WHITE_ENABLE = 0
    VCNL4040_WHITE_DISABLE = (1 << 7)

    VCNL4040_PS_MS_MASK = (~(1 << 6)) & 0xff
    VCNL4040_PS_MS_DISABLE = 0
    VCNL4040_PS_MS_ENABLE = (1 << 6)

    VCNL4040_LED_I_MASK = (~((1 << 2) | (1 << 1) | (1 << 0))) & 0xff
    VCNL4040_LED_50MA = 0
    VCNL4040_LED_75MA = (1 << 0)
    VCNL4040_LED_100MA = (1 << 1)
    VCNL4040_LED_120MA = (1 << 1) | (1 << 0)
    VCNL4040_LED_140MA = (1 << 2)
    VCNL4040_LED_160MA = (1 << 2) | (1 << 0)
    VCNL4040_LED_180MA = (1 << 2) | (1 << 1)
    VCNL4040_LED_200MA = (1 << 2) | (1 << 1) | (1 << 0)

    VCNL4040_INT_FLAG_ALS_LOW = (1 << 5)
    VCNL4040_INT_FLAG_ALS_HIGH = (1 << 4)
    VCNL4040_INT_FLAG_CLOSE = (1 << 1)
    VCNL4040_INT_FLAG_AWAY = (1 << 0)

    def __init__(self, address=None, bus_number=0):
        self._bus = SMBus(bus_number)    
        self._address = address if address is not None else self.available_addresses[0]


    #----------------------------------------------
    # //Check comm with sensor and set it to default init settings
    def begin(self):
        """
            Initialize the operation of the Proximity module

            :return: Returns true of the initializtion was successful, otherwise False.
            :rtype: bool

        """

        if self.get_id() != 0x0186:
            return False  # Check default ID value

        # //Configure the various parts of the sensor
        self.set_led_current(200) # Max IR LED current

        self.set_ir_dutycycle(40) # Set to highest duty cycle

        self.set_prox_integration_time(8) # Set to max integration

        self.set_prox_resolution(16) # Set to 16-bit output

        self.enable_smart_persistance() #Turn on smart presistance

        self.power_on_proximity() #Turn on prox sensing

        self.set_ambient_integration_time(self.VCNL4040_ALS_IT_80MS) #Keep it short
        self.power_on_ambient() #Turn on ambient sensing

        return True

    #----------------------------------------------
    # //Set the duty cycle of the IR LED. The higher the duty
    # //ratio, the faster the response time achieved with higher power
    # //consumption. For example, PS_Duty = 1/320, peak IRED current = 100 mA,
    # //averaged current consumption is 100 mA/320 = 0.3125 mA.
    def set_ir_dutycycle(self, dutyValue):
        """
            Set the duty cycle of the IR LED. The higher the duty
            ratio, the faster the response time achieved with higher power
            consumption. For example, PS_Duty = 1/320, peak IRED current = 100 mA,
            averaged current consumption is 100 mA/320 = 0.3125 mA.

            :param dutyValue: The duty cycle value for the IR LED on the sensor
            :return: No return value

        """
        if dutyValue > 320 - 1:
            dutyValue = self.VCNL4040_PS_DUTY_320
        elif dutyValue > 160 - 1:
            dutyValue = self.VCNL4040_PS_DUTY_160
        elif dutyValue > 80 - 1:
            dutyValue = self.VCNL4040_PS_DUTY_80
        else:
            dutyValue = self.VCNL4040_PS_DUTY_40

        self._bitMask(VCNL4040_PS_CONF1, LOWER, self.VCNL4040_PS_DUTY_MASK, dutyValue)

    duty_cycle = property()
    duty_cycle = duty_cycle.setter(set_ir_dutycycle)

    

    # Power on or off the ambient light sensing portion of the sensor
    def power_on_ambient(self):
        """
            Power on the ambient light sensing portion of the sensor

            :return: No return value
        """
        self._bitMask(VCNL4040_ALS_CONF, LOWER, self.VCNL4040_ALS_SD_MASK, self.VCNL4040_ALS_SD_POWER_ON)

    def power_off_ambient(self):
        """
            Power off the ambient light sensing portion of the sensor

            :return: No return value
        """
        self._bitMask(VCNL4040_ALS_CONF, LOWER, self.VCNL4040_ALS_SD_MASK, self.VCNL4040_ALS_SD_POWER_OFF)

    # Sets the integration time for the ambient light sensor
    def set_ambient_integration_time(self, timeValue):
        """
            Sets the integration time for the ambient light sensor

            :param timeValue: The integration time
            :return: No return value
        """

        if timeValue > 640 - 1:
            timeValue = self.VCNL4040_ALS_IT_640MS
        elif timeValue > 320 - 1:
            timeValue = self.VCNL4040_ALS_IT_320MS
        elif timeValue > 160 - 1:
            timeValue = self.VCNL4040_ALS_IT_160MS
        else:
            timeValue = self.VCNL4040_ALS_IT_80MS

        self._bitMask(VCNL4040_ALS_CONF, LOWER, self.VCNL4040_ALS_IT_MASK, timeValue)

    ambient_integration_time = property()
    ambient_integration_time = ambient_integration_time.setter(set_ambient_integration_time)

    # Sets the integration time for the proximity sensor
    def set_prox_integration_time(self, timeValue):
        """
            Sets the integration time for the proximity sensor

            :param timeValue: The integration time
            :return: No return value
        """
        if timeValue > 8 - 1:
            timeValue = self.VCNL4040_PS_IT_8T
        elif timeValue > 4 - 1:
            timeValue = self.VCNL4040_PS_IT_4T
        elif timeValue > 3 - 1:
            timeValue = self.VCNL4040_PS_IT_3T
        elif timeValue > 2 - 1:
            timeValue = self.VCNL4040_PS_IT_2T
        else:
            timeValue = self.VCNL4040_PS_IT_1T

        self._bitMask(VCNL4040_PS_CONF1, LOWER, self.VCNL4040_PS_IT_MASK, timeValue)

    prox_integration_time = property()
    prox_integration_time = prox_integration_time.setter(set_prox_integration_time)

    # Power on the prox sensing portion of the device
    def power_on_proximity(self):
        """
            Power on the prox sensing portion of the device

            :return: No return value
        """
        self._bitMask(VCNL4040_PS_CONF1, LOWER, self.VCNL4040_PS_SD_MASK, self.VCNL4040_PS_SD_POWER_ON)

    # Power off the prox sensing portion of the device
    def power_off_proximity(self):
        """
            Power off the prox sensing portion of the device

            :return: No return value
        """
        self._bitMask(VCNL4040_PS_CONF1, LOWER, self.VCNL4040_PS_SD_MASK, self.VCNL4040_PS_SD_POWER_OFF)

    # Sets the proximity resolution
    def set_prox_resolution(self, resolutionValue):
        """
            Sets the proximity resolution

            :param resolutionValue: The resolution time
            :return: No return value
        """
        if resolutionValue > 16 - 1:
            resolutionValue = self.VCNL4040_PS_HD_16_BIT
        else:
            resolutionValue = self.VCNL4040_PS_HD_12_BIT

        self._bitMask(VCNL4040_PS_CONF2, UPPER, self.VCNL4040_PS_HD_MASK, resolutionValue)

    prox_resolution = property()
    prox_resolution = prox_resolution.setter(set_prox_resolution)


    # Enable smart persistance
    # To accelerate the PS response time, smart
    # persistence prevents the misjudgment of proximity sensing
    # but also keeps a fast response time.
    def enable_smart_persistance(self):
        """
            Enable smart persistance
            To accelerate the PS response time, smart
            persistence prevents the misjudgment of proximity sensing
            but also keeps a fast response time.

            :return: No return value
        """
        self._bitMask(VCNL4040_PS_CONF3, LOWER, self.VCNL4040_PS_SMART_PERS_MASK, self.VCNL4040_PS_SMART_PERS_ENABLE)

    def disable_smart_persistance(self):
        """
            Disable smart persistance

            :return: No return value
        """
        self._bitMask(VCNL4040_PS_CONF3, LOWER, self.VCNL4040_PS_SMART_PERS_MASK, self.VCNL4040_PS_SMART_PERS_DISABLE)


    # Set the IR LED sink current to one of 8 settings
    def set_led_current(self, currentValue):
        """
            Set the IR LED sink current to one of 8 settings

            :param currentValue: The new current value. Valid values are VCNL4040_LED_50MA thru VCNL4040_LED_200MA at 25MA increments
            :return: No return value
        """

        if currentValue > 200 - 1:
            currentValue = self.VCNL4040_LED_200MA
        elif currentValue > 180 - 1:
            currentValue = self.VCNL4040_LED_180MA
        elif currentValue > 160 - 1:
            currentValue = self.VCNL4040_LED_160MA
        elif currentValue > 140 - 1:
            currentValue = self.VCNL4040_LED_140MA
        elif currentValue > 120 - 1:
            currentValue = self.VCNL4040_LED_120MA
        elif currentValue > 100 - 1:
            currentValue = self.VCNL4040_LED_100MA
        elif currentValue > 75 - 1:
            currentValue = self.VCNL4040_LED_75MA
        else:
            currentValue = self.VCNL4040_LED_50MA

        self._bitMask(VCNL4040_PS_MS, UPPER, self.VCNL4040_LED_I_MASK, currentValue)

    led_current = property()
    led_current = led_current.setter(set_led_current)


    #--------------------------------------------------------------------------
    # Read Value methods
    #--------------------------------------------------------------------------
    # get_proximity()
    #
    # Read the Proximity value
    def get_proximity(self):
        """
            Get the current proximity value

            :return: The current proximity value
            :rtype: integer
        """
        # return self._i2c.readWord(self.address, VCNL4040_PS_DATA)
        return self._bus.read_word_data(self._address, VCNL4040_PS_DATA)


    # prox as a readonly prop
    proximity = property(get_proximity)

    # Read the Ambient light value
    def get_ambient(self):
        """
            Read the Ambient light value

            :return: The current ambient value value
            :rtype: integer
        """
        # return self._i2c.readWord(self.address, VCNL4040_ALS_DATA)
        return self._bus.read_word_data(self._address, VCNL4040_ALS_DATA)

    # Ambient as a readonly prop
    ambient = property(get_ambient)


    # Read the Whilte light value
    def get_white(self):
        """
            Read the White light value

            :return: The current white value value
            :rtype: integer
        """
        return self._bus.read_word_data(self._address, VCNL4040_WHITE_DATA)
        # return self._i2c.readWord(self.address, VCNL4040_WHITE_DATA)

    # White light as a readonly prop
    white_light = property(get_white)


    # Read the sensors ID
    def get_id(self):
        """
            Read the sensor ID

            :return: The sensor ID
            :rtype: integer
        """
        return self._bus.read_word_data(self._address, VCNL4040_ID)
        # return self._i2c.readWord(self.address, VCNL4040_ID)

    # Sensor ID as a readonly prop
    sensor_id = property(get_id)





    #--------------------------------------------------------------------------
    # internal I2C Utility Routines
    #--------------------------------------------------------------------------

    #--------------------------------------------------------------------------
    # _writeCommandLower()
    #
    # Given a command code (address) write to the upper byte without affecting the upper byte
    def _writeCommandLower(self, commandCode, newValue):

        commandValue = self._bus.read_word_data(self._address, commandCode)
        commandValue &= 0xFF00    #Remove lower 8 bits
        commandValue |= newValue    #Mask in
        return self._bus.write_word_data(self._address, commandCode, commandValue)
    #--------------------------------------------------------------------------
    # _writeCommandUpper()
    #
    # Given a command code (address) write to the upper byte without affecting the lower byte
    def _writeCommandUpper(self, commandCode, newValue):

        commandValue = self._bus.read_word_data(self._address, commandCode)
        commandValue &= 0x00FF    #Remove upper 8 bits
        commandValue |= newValue << 8   #Mask in

        return self._bus.write_word_data(self._address, commandCode, commandValue)
    #--------------------------------------------------------------------------
    # _readCommandLower()
    #
    # Given a command code (address) read the lower byte
    def _readCommandLower(self, commandCode):

        commandValue = self._bus.read_word_data(self._address, commandCode)

        return commandValue & 0xFF

    #--------------------------------------------------------------------------
    # _readCommandUpper()
    #
    # Given a command code (address) read the upper byte
    def _readCommandUpper(self, commandCode):

        commandValue = self._bus.read_word_data(self._address, commandCode)

        return commandValue >> 8

    #--------------------------------------------------------------------------
    # _bitMask()
    #
    # Given a register, read it, mask it, and then set the thing
    # commandHeight is used to select between the upper or lower byte of command register
    # Example:
    # Write dutyValue into PS_CONF1, lower byte, using the Duty_Mask
    # _bitMask(VCNL4040_PS_CONF1, LOWER, VCNL4040_PS_DUTY_MASK, dutyValue);
    def _bitMask(self, commandAddress, commandHeight, mask, thing):

        # Grab current register context
        if commandHeight == LOWER:
            registerContents = self._readCommandLower(commandAddress)
        else:
            registerContents = self._readCommandUpper(commandAddress)

        # Zero-out the portions of the register we're interested in
        registerContents &= mask

        # Mask in new thing
        registerContents |= thing

        # Change contents
        if commandHeight == LOWER:
            self._writeCommandLower(commandAddress, registerContents)
        else:
            self._writeCommandUpper(commandAddress, registerContents)
            
class QwiicProximity1(object):
    """
    QwiicProximity

        :param address: The I2C address to use for the device.
                        If not provided, the default address is used.
        :param i2c_driver: An existing i2c driver object. If not provided
                        a driver object is created.
        :return: The Proximity device object.
        :rtype: Object
    """

    # Constructor
    device_name = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    # Flags exposed in userspace - make these class vars
    VCNL4040_PS_INT_MASK = (~((1 << 1) | (1 << 0))) & 0xff
    VCNL4040_PS_INT_DISABLE = 0
    VCNL4040_PS_INT_CLOSE = (1 << 0)
    VCNL4040_PS_INT_AWAY = (1 << 1)
    VCNL4040_PS_INT_BOTH = (1 << 1) | (1 << 0)

    # Define commands/commandwords for the sensor
    VCNL4040_ALS_IT_MASK = (~((1 << 7) | (1 << 6))) & 0xff #uint8
    VCNL4040_ALS_IT_80MS = 0
    VCNL4040_ALS_IT_160MS = (1 << 7)
    VCNL4040_ALS_IT_320MS = (1 << 6)
    VCNL4040_ALS_IT_640MS = (1 << 7) | (1 << 6)

    VCNL4040_ALS_PERS_MASK = (~((1 << 3) | (1 << 2))) & 0xff
    VCNL4040_ALS_PERS_1 = 0
    VCNL4040_ALS_PERS_2 = (1 << 2)
    VCNL4040_ALS_PERS_4 = (1 << 3)
    VCNL4040_ALS_PERS_8 = (1 << 3) | (1 << 2)

    VCNL4040_ALS_INT_EN_MASK = (~(1 << 1)) & 0xff
    VCNL4040_ALS_INT_DISABLE = 0
    VCNL4040_ALS_INT_ENABLE = (1 << 1)

    VCNL4040_ALS_SD_MASK = (~(1 << 0)) & 0xff
    VCNL4040_ALS_SD_POWER_ON = 0
    VCNL4040_ALS_SD_POWER_OFF = (1 << 0)

    VCNL4040_PS_DUTY_MASK = (~((1 << 7) | (1 << 6))) & 0xff
    VCNL4040_PS_DUTY_40 = 0
    VCNL4040_PS_DUTY_80 = (1 << 6)
    VCNL4040_PS_DUTY_160 = (1 << 7)
    VCNL4040_PS_DUTY_320 = (1 << 7) | (1 << 6)

    VCNL4040_PS_PERS_MASK = (~((1 << 5) | (1 << 4))) & 0xff
    VCNL4040_PS_PERS_1 = 0
    VCNL4040_PS_PERS_2 = (1 << 4)
    VCNL4040_PS_PERS_3 = (1 << 5)
    VCNL4040_PS_PERS_4 = (1 << 5) | (1 << 4)

    VCNL4040_PS_IT_MASK = (~((1 << 3) | (1 << 2) | (1 << 1))) & 0xff
    VCNL4040_PS_IT_1T = 0
    VCNL4040_PS_IT_15T = (1 << 1)
    VCNL4040_PS_IT_2T = (1 << 2)
    VCNL4040_PS_IT_25T = (1 << 2) | (1 << 1)
    VCNL4040_PS_IT_3T = (1 << 3)
    VCNL4040_PS_IT_35T = (1 << 3) | (1 << 1)
    VCNL4040_PS_IT_4T = (1 << 3) | (1 << 2)
    VCNL4040_PS_IT_8T = (1 << 3) | (1 << 2) | (1 << 1)

    VCNL4040_PS_SD_MASK = (~(1 << 0)) & 0xff
    VCNL4040_PS_SD_POWER_ON = 0
    VCNL4040_PS_SD_POWER_OFF = (1 << 0)

    VCNL4040_PS_HD_MASK = (~(1 << 3)) & 0xff
    VCNL4040_PS_HD_12_BIT = 0
    VCNL4040_PS_HD_16_BIT = (1 << 3)

    VCNL4040_PS_SMART_PERS_MASK = (~(1 << 4)) & 0xff
    VCNL4040_PS_SMART_PERS_DISABLE = 0
    VCNL4040_PS_SMART_PERS_ENABLE = (1 << 1)

    VCNL4040_PS_AF_MASK = (~(1 << 3)) & 0xff
    VCNL4040_PS_AF_DISABLE = 0
    VCNL4040_PS_AF_ENABLE = (1 << 3)

    VCNL4040_PS_TRIG_MASK = (~(1 << 3)) & 0xff
    VCNL4040_PS_TRIG_TRIGGER = (1 << 2)

    VCNL4040_WHITE_EN_MASK = (~(1 << 7)) & 0xff
    VCNL4040_WHITE_ENABLE = 0
    VCNL4040_WHITE_DISABLE = (1 << 7)

    VCNL4040_PS_MS_MASK = (~(1 << 6)) & 0xff
    VCNL4040_PS_MS_DISABLE = 0
    VCNL4040_PS_MS_ENABLE = (1 << 6)

    VCNL4040_LED_I_MASK = (~((1 << 2) | (1 << 1) | (1 << 0))) & 0xff
    VCNL4040_LED_50MA = 0
    VCNL4040_LED_75MA = (1 << 0)
    VCNL4040_LED_100MA = (1 << 1)
    VCNL4040_LED_120MA = (1 << 1) | (1 << 0)
    VCNL4040_LED_140MA = (1 << 2)
    VCNL4040_LED_160MA = (1 << 2) | (1 << 0)
    VCNL4040_LED_180MA = (1 << 2) | (1 << 1)
    VCNL4040_LED_200MA = (1 << 2) | (1 << 1) | (1 << 0)

    VCNL4040_INT_FLAG_ALS_LOW = (1 << 5)
    VCNL4040_INT_FLAG_ALS_HIGH = (1 << 4)
    VCNL4040_INT_FLAG_CLOSE = (1 << 1)
    VCNL4040_INT_FLAG_AWAY = (1 << 0)

    def __init__(self, address=None, bus_number=1):
        self._bus = SMBus(bus_number)    
        self._address = address if address is not None else self.available_addresses[0]


    #----------------------------------------------
    # //Check comm with sensor and set it to default init settings
    def begin(self):
        """
            Initialize the operation of the Proximity module

            :return: Returns true of the initializtion was successful, otherwise False.
            :rtype: bool

        """

        if self.get_id() != 0x0186:
            return False  # Check default ID value

        # //Configure the various parts of the sensor
        self.set_led_current(200) # Max IR LED current

        self.set_ir_dutycycle(40) # Set to highest duty cycle

        self.set_prox_integration_time(8) # Set to max integration

        self.set_prox_resolution(16) # Set to 16-bit output

        self.enable_smart_persistance() #Turn on smart presistance

        self.power_on_proximity() #Turn on prox sensing

        self.set_ambient_integration_time(self.VCNL4040_ALS_IT_80MS) #Keep it short
        self.power_on_ambient() #Turn on ambient sensing

        return True

    #----------------------------------------------
    # //Set the duty cycle of the IR LED. The higher the duty
    # //ratio, the faster the response time achieved with higher power
    # //consumption. For example, PS_Duty = 1/320, peak IRED current = 100 mA,
    # //averaged current consumption is 100 mA/320 = 0.3125 mA.
    def set_ir_dutycycle(self, dutyValue):
        """
            Set the duty cycle of the IR LED. The higher the duty
            ratio, the faster the response time achieved with higher power
            consumption. For example, PS_Duty = 1/320, peak IRED current = 100 mA,
            averaged current consumption is 100 mA/320 = 0.3125 mA.

            :param dutyValue: The duty cycle value for the IR LED on the sensor
            :return: No return value

        """
        if dutyValue > 320 - 1:
            dutyValue = self.VCNL4040_PS_DUTY_320
        elif dutyValue > 160 - 1:
            dutyValue = self.VCNL4040_PS_DUTY_160
        elif dutyValue > 80 - 1:
            dutyValue = self.VCNL4040_PS_DUTY_80
        else:
            dutyValue = self.VCNL4040_PS_DUTY_40

        self._bitMask(VCNL4040_PS_CONF1, LOWER, self.VCNL4040_PS_DUTY_MASK, dutyValue)

    duty_cycle = property()
    duty_cycle = duty_cycle.setter(set_ir_dutycycle)

    

    # Power on or off the ambient light sensing portion of the sensor
    def power_on_ambient(self):
        """
            Power on the ambient light sensing portion of the sensor

            :return: No return value
        """
        self._bitMask(VCNL4040_ALS_CONF, LOWER, self.VCNL4040_ALS_SD_MASK, self.VCNL4040_ALS_SD_POWER_ON)

    def power_off_ambient(self):
        """
            Power off the ambient light sensing portion of the sensor

            :return: No return value
        """
        self._bitMask(VCNL4040_ALS_CONF, LOWER, self.VCNL4040_ALS_SD_MASK, self.VCNL4040_ALS_SD_POWER_OFF)

    # Sets the integration time for the ambient light sensor
    def set_ambient_integration_time(self, timeValue):
        """
            Sets the integration time for the ambient light sensor

            :param timeValue: The integration time
            :return: No return value
        """

        if timeValue > 640 - 1:
            timeValue = self.VCNL4040_ALS_IT_640MS
        elif timeValue > 320 - 1:
            timeValue = self.VCNL4040_ALS_IT_320MS
        elif timeValue > 160 - 1:
            timeValue = self.VCNL4040_ALS_IT_160MS
        else:
            timeValue = self.VCNL4040_ALS_IT_80MS

        self._bitMask(VCNL4040_ALS_CONF, LOWER, self.VCNL4040_ALS_IT_MASK, timeValue)

    ambient_integration_time = property()
    ambient_integration_time = ambient_integration_time.setter(set_ambient_integration_time)

    # Sets the integration time for the proximity sensor
    def set_prox_integration_time(self, timeValue):
        """
            Sets the integration time for the proximity sensor

            :param timeValue: The integration time
            :return: No return value
        """
        if timeValue > 8 - 1:
            timeValue = self.VCNL4040_PS_IT_8T
        elif timeValue > 4 - 1:
            timeValue = self.VCNL4040_PS_IT_4T
        elif timeValue > 3 - 1:
            timeValue = self.VCNL4040_PS_IT_3T
        elif timeValue > 2 - 1:
            timeValue = self.VCNL4040_PS_IT_2T
        else:
            timeValue = self.VCNL4040_PS_IT_1T

        self._bitMask(VCNL4040_PS_CONF1, LOWER, self.VCNL4040_PS_IT_MASK, timeValue)

    prox_integration_time = property()
    prox_integration_time = prox_integration_time.setter(set_prox_integration_time)

    # Power on the prox sensing portion of the device
    def power_on_proximity(self):
        """
            Power on the prox sensing portion of the device

            :return: No return value
        """
        self._bitMask(VCNL4040_PS_CONF1, LOWER, self.VCNL4040_PS_SD_MASK, self.VCNL4040_PS_SD_POWER_ON)

    # Power off the prox sensing portion of the device
    def power_off_proximity(self):
        """
            Power off the prox sensing portion of the device

            :return: No return value
        """
        self._bitMask(VCNL4040_PS_CONF1, LOWER, self.VCNL4040_PS_SD_MASK, self.VCNL4040_PS_SD_POWER_OFF)

    # Sets the proximity resolution
    def set_prox_resolution(self, resolutionValue):
        """
            Sets the proximity resolution

            :param resolutionValue: The resolution time
            :return: No return value
        """
        if resolutionValue > 16 - 1:
            resolutionValue = self.VCNL4040_PS_HD_16_BIT
        else:
            resolutionValue = self.VCNL4040_PS_HD_12_BIT

        self._bitMask(VCNL4040_PS_CONF2, UPPER, self.VCNL4040_PS_HD_MASK, resolutionValue)

    prox_resolution = property()
    prox_resolution = prox_resolution.setter(set_prox_resolution)


    # Enable smart persistance
    # To accelerate the PS response time, smart
    # persistence prevents the misjudgment of proximity sensing
    # but also keeps a fast response time.
    def enable_smart_persistance(self):
        """
            Enable smart persistance
            To accelerate the PS response time, smart
            persistence prevents the misjudgment of proximity sensing
            but also keeps a fast response time.

            :return: No return value
        """
        self._bitMask(VCNL4040_PS_CONF3, LOWER, self.VCNL4040_PS_SMART_PERS_MASK, self.VCNL4040_PS_SMART_PERS_ENABLE)

    def disable_smart_persistance(self):
        """
            Disable smart persistance

            :return: No return value
        """
        self._bitMask(VCNL4040_PS_CONF3, LOWER, self.VCNL4040_PS_SMART_PERS_MASK, self.VCNL4040_PS_SMART_PERS_DISABLE)


    # Set the IR LED sink current to one of 8 settings
    def set_led_current(self, currentValue):
        """
            Set the IR LED sink current to one of 8 settings

            :param currentValue: The new current value. Valid values are VCNL4040_LED_50MA thru VCNL4040_LED_200MA at 25MA increments
            :return: No return value
        """

        if currentValue > 200 - 1:
            currentValue = self.VCNL4040_LED_200MA
        elif currentValue > 180 - 1:
            currentValue = self.VCNL4040_LED_180MA
        elif currentValue > 160 - 1:
            currentValue = self.VCNL4040_LED_160MA
        elif currentValue > 140 - 1:
            currentValue = self.VCNL4040_LED_140MA
        elif currentValue > 120 - 1:
            currentValue = self.VCNL4040_LED_120MA
        elif currentValue > 100 - 1:
            currentValue = self.VCNL4040_LED_100MA
        elif currentValue > 75 - 1:
            currentValue = self.VCNL4040_LED_75MA
        else:
            currentValue = self.VCNL4040_LED_50MA

        self._bitMask(VCNL4040_PS_MS, UPPER, self.VCNL4040_LED_I_MASK, currentValue)

    led_current = property()
    led_current = led_current.setter(set_led_current)


    #--------------------------------------------------------------------------
    # Read Value methods
    #--------------------------------------------------------------------------
    # get_proximity()
    #
    # Read the Proximity value
    def get_proximity(self):
        """
            Get the current proximity value

            :return: The current proximity value
            :rtype: integer
        """
        # return self._i2c.readWord(self.address, VCNL4040_PS_DATA)
        return self._bus.read_word_data(self._address, VCNL4040_PS_DATA)


    # prox as a readonly prop
    proximity = property(get_proximity)

    # Read the Ambient light value
    def get_ambient(self):
        """
            Read the Ambient light value

            :return: The current ambient value value
            :rtype: integer
        """
        # return self._i2c.readWord(self.address, VCNL4040_ALS_DATA)
        return self._bus.read_word_data(self._address, VCNL4040_ALS_DATA)

    # Ambient as a readonly prop
    ambient = property(get_ambient)


    # Read the Whilte light value
    def get_white(self):
        """
            Read the White light value

            :return: The current white value value
            :rtype: integer
        """
        return self._bus.read_word_data(self._address, VCNL4040_WHITE_DATA)
        # return self._i2c.readWord(self.address, VCNL4040_WHITE_DATA)

    # White light as a readonly prop
    white_light = property(get_white)


    # Read the sensors ID
    def get_id(self):
        """
            Read the sensor ID

            :return: The sensor ID
            :rtype: integer
        """
        return self._bus.read_word_data(self._address, VCNL4040_ID)
        # return self._i2c.readWord(self.address, VCNL4040_ID)

    # Sensor ID as a readonly prop
    sensor_id = property(get_id)





    #--------------------------------------------------------------------------
    # internal I2C Utility Routines
    #--------------------------------------------------------------------------

    #--------------------------------------------------------------------------
    # _writeCommandLower()
    #
    # Given a command code (address) write to the upper byte without affecting the upper byte
    def _writeCommandLower(self, commandCode, newValue):

        commandValue = self._bus.read_word_data(self._address, commandCode)
        commandValue &= 0xFF00    #Remove lower 8 bits
        commandValue |= newValue    #Mask in
        return self._bus.write_word_data(self._address, commandCode, commandValue)
    #--------------------------------------------------------------------------
    # _writeCommandUpper()
    #
    # Given a command code (address) write to the upper byte without affecting the lower byte
    def _writeCommandUpper(self, commandCode, newValue):

        commandValue = self._bus.read_word_data(self._address, commandCode)
        commandValue &= 0x00FF    #Remove upper 8 bits
        commandValue |= newValue << 8   #Mask in

        return self._bus.write_word_data(self._address, commandCode, commandValue)
    #--------------------------------------------------------------------------
    # _readCommandLower()
    #
    # Given a command code (address) read the lower byte
    def _readCommandLower(self, commandCode):

        commandValue = self._bus.read_word_data(self._address, commandCode)

        return commandValue & 0xFF

    #--------------------------------------------------------------------------
    # _readCommandUpper()
    #
    # Given a command code (address) read the upper byte
    def _readCommandUpper(self, commandCode):

        commandValue = self._bus.read_word_data(self._address, commandCode)

        return commandValue >> 8

    #--------------------------------------------------------------------------
    # _bitMask()
    #
    # Given a register, read it, mask it, and then set the thing
    # commandHeight is used to select between the upper or lower byte of command register
    # Example:
    # Write dutyValue into PS_CONF1, lower byte, using the Duty_Mask
    # _bitMask(VCNL4040_PS_CONF1, LOWER, VCNL4040_PS_DUTY_MASK, dutyValue);
    def _bitMask(self, commandAddress, commandHeight, mask, thing):

        # Grab current register context
        if commandHeight == LOWER:
            registerContents = self._readCommandLower(commandAddress)
        else:
            registerContents = self._readCommandUpper(commandAddress)

        # Zero-out the portions of the register we're interested in
        registerContents &= mask

        # Mask in new thing
        registerContents |= thing

        # Change contents
        if commandHeight == LOWER:
            self._writeCommandLower(commandAddress, registerContents)
        else:
            self._writeCommandUpper(commandAddress, registerContents)
