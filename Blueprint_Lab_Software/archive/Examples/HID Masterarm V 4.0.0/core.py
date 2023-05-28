import pywinusb.hid as hid

class Masterarm:

    def __init__(self):
        self.channels = []
        self.indicator_state = [0, 0, 0]   # Set the current state to off

        # Filter HID devices for master arm via vendor ID and product ID and get masterarm channels
        hid_devices = hid.HidDeviceFilter(VendorId=Enum.STM32_VID, product_id=Enum.MASTERARM_PID)
        masterarm_channels = hid_devices.get_devices()

        # Master arm will have 3 HID device channels. Record channels in list.
        # If non masterarm device is detected, then throw IO error.
        if len(masterarm_channels) == 3:
            for idx, new_channel in enumerate(masterarm_channels):
                self.channels.append(Channel(new_channel, idx))
                self.channels[idx].set_connected()
        else:
            raise IOError("No masterarm connected")

    def show_hids(self):
        """Print a report with all HID devices connected"""
        hid.core.show_hids()


class Channel:

    def __init__(self, device, idx):
        self.channel_idx = idx          # Keep track of the number of channels
        self.axis_value = []            # Empty list of axis values
        self.inputs_state = []          # Empty list of inputs
        self.outputs_state = []         # Empty list of outputs

        self.device = device            # Assign device to current instance
        self.device.open()              # Open device for communication

        self.input_reports = self.device.find_input_reports(usage_id=idx)   # Find input reports for current device

        self.device.set_raw_data_handler(self.masterarm_inputs_handler)     # Setup handler to poll masterarm status
        self.out_report = self.device.find_output_reports()                 # Create an output_report object
        self.__state_connected = False                                      # Initialize as disconnected

    def set_connected(self):
        """set connection status true by calling this method"""
        self.__state_connected = True

    def close(self):
        """close connection by calling this method"""
        if self.__state_connected:
            self.clear_outputs()
            self.device.close()     # close the connection and kill the thread
            del self                # delete the object
            self.__state_connected = False

    def clear_outputs(self):
        """clear all outputs"""
        self.outputs_state = [0, 0, 0, 0]
        buffer = [0] + self.outputs_state  # first item of the list is ID=0x00
        self.out_report[0].set_raw_data(buffer)
        self.out_report[0].send()

    @staticmethod
    def convert(hi_byte, lo_byte):
        """convert two bytes into 16 bit integer"""
        return (hi_byte << 8) + lo_byte

    def masterarm_inputs_handler(self, data):
        """process input data for each channel"""
        self.axis_value = []

        if data[0] == Enum.HANDLE + 1:
            # Buttons [Stow, Deploy, Pause]
            self.inputs_state = [int(x) for x in bin(data[1])[2:].zfill(3)]  # convert int value to binary list

            # Joystick axis [X, Y]
            self.axis_value.append(2*self.convert(data[3], data[2])/Enum.AXIS_SCALE-1)
            self.axis_value.append(2*self.convert(data[5], data[4])/Enum.AXIS_SCALE-1)
        if data[0] == Enum.JOINT_VELOCITY + 1:
            # Velocity axis [B, C, D, E, F, G]
            self.axis_value.append(Enum.VELOCITY_SCALE*(self.convert(data[3], data[2])-Enum.AXIS_SCALE/2)/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.VELOCITY_SCALE*(self.convert(data[5], data[4])-Enum.AXIS_SCALE/2)/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.VELOCITY_SCALE*(self.convert(data[7], data[6])-Enum.AXIS_SCALE/2)/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.VELOCITY_SCALE*(self.convert(data[9], data[8])-Enum.AXIS_SCALE/2)/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.VELOCITY_SCALE*(self.convert(data[11], data[10])-Enum.AXIS_SCALE/2)/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.VELOCITY_SCALE*(self.convert(data[13], data[12])-Enum.AXIS_SCALE/2)/Enum.AXIS_SCALE)
        if data[0] == Enum.JOINT_POSITION + 1:
            # Position axis [B, C, D, E, F, G]
            self.axis_value.append(Enum.POSITION_SCALE*self.convert(data[3], data[2])/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.POSITION_SCALE*self.convert(data[5], data[4])/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.POSITION_SCALE*self.convert(data[7], data[6])/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.POSITION_SCALE*self.convert(data[9], data[8])/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.POSITION_SCALE*self.convert(data[11], data[10])/Enum.AXIS_SCALE)
            self.axis_value.append(Enum.POSITION_SCALE*self.convert(data[13], data[12])/Enum.AXIS_SCALE)

    def get_axis(self):
        """return the value of the axis 0..2 / return signed integer """
        return self.axis_value

    def get_input(self):
        """return the state of the input 0..15 / return 0 or 1 """
        return self.inputs_state

    def send_out_report(self, data):
        masterarm_out = [0x00] * Enum.LEN
        for i in range(len(data)):
            masterarm_out[i + 1] = data[i]  # Must reserve first byte as this is used by USB hardware

        self.out_report[0].set_raw_data(masterarm_out)
        self.out_report[0].send()


class Enum:
    """ Device enumerations """
    # Number of buttons
    NUM_BUTTONS = 3

    # Scaling
    VELOCITY_SCALE = 10
    POSITION_SCALE = 360

    # Button ID's
    PAUSE = 0x02
    STOW = 0x01
    DEPLOY = 0x00

    # LED ID's
    LED_PAUSE = 0x01
    LED_STOW = 0x02
    LED_DEPLOY = 0x03

    # Led's outputs values
    OFF = 0x00  # Off
    ON = 0x01  # On
    FAST_FLASH = 0x02  # Fast blink mode
    SLOW_FLASH = 0x03  # Slow blink mode

    # HID message ID
    ID = 0x00

    # AXIS ID's
    X = 0x00
    Y = 0x01
    B = 0x00
    C = 0x01
    D = 0x02
    E = 0x03
    F = 0x04
    G = 0x05

    # Channel types
    HANDLE = 0x00
    JOINT_VELOCITY = 0x01
    JOINT_POSITION = 0x02

    # Axis scale
    AXIS_SCALE = 65534.0

    # Byte length
    LEN = 65

    # USB Hardware ID's
    STM32_VID = 1155
    MASTERARM_PID = 22352
