from time import sleep
from core import Enum, Masterarm, Channel

class UserMethods:
    PAUSED = 0
    RUNNING = 1

    def __init__(self):
        self.mode = None
        self.toggled_on = [0] * Enum.NUM_BUTTONS
        self.toggled_off = [0] * Enum.NUM_BUTTONS
        self.buttons = []

    @staticmethod
    def slow_flash_all(masterarm):
        """ Slow flash all button indicators """
        UserMethods.reset_all(masterarm)  # Reset all to ensure LED's blink at same rate
        masterarm.channels[0].send_out_report([Enum.LED_PAUSE, Enum.SLOW_FLASH])
        masterarm.channels[0].send_out_report([Enum.LED_STOW, Enum.SLOW_FLASH])
        masterarm.channels[0].send_out_report([Enum.LED_DEPLOY, Enum.SLOW_FLASH])

        # Set the new indicator state to all slow flash
        masterarm.indicator_state = [Enum.SLOW_FLASH] * Enum.NUM_BUTTONS

    @staticmethod
    def fast_flash_all(masterarm):
        """ Fast flash all button indicators """
        UserMethods.reset_all(masterarm)  # Reset all to ensure LED's blink at same rate
        masterarm.channels[0].send_out_report([Enum.LED_PAUSE, Enum.FAST_FLASH])
        masterarm.channels[0].send_out_report([Enum.LED_STOW, Enum.FAST_FLASH])
        masterarm.channels[0].send_out_report([Enum.LED_DEPLOY, Enum.FAST_FLASH])

        # Set the new indicator state to all fast flash
        masterarm.indicator_state = [Enum.FAST_FLASH] * Enum.NUM_BUTTONS

    @staticmethod
    def reset_all(masterarm):
        """ Reset all button indicators (all off) """
        masterarm.channels[0].send_out_report([Enum.LED_PAUSE, Enum.OFF])
        masterarm.channels[0].send_out_report([Enum.LED_STOW, Enum.OFF])
        masterarm.channels[0].send_out_report([Enum.LED_DEPLOY, Enum.OFF])

        # Set the new indicator state to all off
        masterarm.indicator_state = [Enum.OFF] * Enum.NUM_BUTTONS

    @staticmethod
    def set_all(masterarm):
        """ Set all button indicators (all on) """
        masterarm.channels[0].send_out_report([Enum.LED_PAUSE, Enum.ON])
        masterarm.channels[0].send_out_report([Enum.LED_STOW, Enum.ON])
        masterarm.channels[0].send_out_report([Enum.LED_DEPLOY, Enum.ON])

        # Set the new indicator state to all on
        masterarm.indicator_state = [Enum.ON] * Enum.NUM_BUTTONS

    @staticmethod
    def set_indicator(masterarm, indicator_id):
        """ Set single button indicator by indiacator ID """
        masterarm.channels[0].send_out_report([indicator_id, Enum.ON])

        # Set the new indicator state
        masterarm.indicator_state[indicator_id - 1] = Enum.ON

    @staticmethod
    def reset_indicator(masterarm, indicator_id):
        """ Reset single button indicator by indiacator ID """
        masterarm.channels[0].send_out_report([indicator_id, Enum.OFF])

        # Set the new indicator state
        masterarm.indicator_state[indicator_id - 1] = Enum.OFF

    @staticmethod
    def fast_flash_indicator(masterarm, indicator_id):
        """ Set single button indicator by indiacator ID """
        masterarm.channels[0].send_out_report([indicator_id, Enum.FAST_FLASH])

        # Set the new indicator state
        masterarm.indicator_state[indicator_id - 1] = Enum.FAST_FLASH

    @staticmethod
    def slow_flash_indicator(masterarm, indicator_id):
        """ Slow flash single button indicator by indiacator ID """
        masterarm.channels[0].send_out_report([indicator_id, Enum.SLOW_FLASH])

        # Set the new indicator state
        masterarm.indicator_state[indicator_id - 1] = Enum.SLOW_FLASH

    def get_button_state(self, channel):
        """ Returns the button state for the current channel.
            If there are no buttons associated with this channel an empty list is returned.
            This empty list has an inherit binary value of zero.
        """
        # Get the button state
        new_buttons = channel.get_input()

        if new_buttons:
            # Reset toggled state
            self.toggled_on = [0] * Enum.NUM_BUTTONS
            self.toggled_off = [0] * Enum.NUM_BUTTONS

            # Determine if button has been toggled since last poll
            for idx, button in enumerate(self.buttons):
                if button == 0 and new_buttons[idx] == 1:
                    self.toggled_on[idx] = 1
                if button == 1 and new_buttons[idx] == 0:
                    self.toggled_off[idx] = 1

            # Set the current button state
            self.buttons = new_buttons

        # return the buttons
        return self.buttons


    @staticmethod
    def get_axis_state(channel):
        """ Returns the button state for the current channel.
            If there are no buttons associated with this channel an empty list is returned.
            This empty list has an inherit binary value of zero.
        """
        return channel.get_axis()

if __name__ == '__main__':
    """ This is an example script for interfacing with a HID masterarm. This script aims at conveying the operation
        of the HID masterarm in a simple manner. Some of the functionality demonstrated in this script is broadly
        stated below; 
                
                - Getting scaled axis values from the joystick along with joint velocities and positions.
                - Getting button states for the pause, stow, and deploy buttons 
                - Sending indicator commands to the pause, stow and deploy LED's
        
        In the following example the this functionality is demonstrated by;
        
                - First instantiating a new HID masterarm class and user methods class
                - Setting the mode to pause and indicating this on the masterarm via a slow flash
                - Entering a while loop that transitions between two modes (PAUSE and RUNNING) based on the "toggled_on"
                  state of the PAUSE button. 
                - When in PAUSE mode; only button states are displayed
                - When in RUNNING mode; button, joystick, joint velocity, and joint position's are all displayed.
                
        Operational notes;
        
                - When masterarm is pluged in bootloader initialization will take 5 seconds
                - After 5 seconds the master arm will become available as a HID device in your computer's device manager
                - To indicate that the device is working correctly in HID mode, the indicator directly below the 
                  joystick will flash fast for 1 second.
                - Indicators will return to the off state when initialization has finished
                - HID master arm will show up as a game controller with three channels in windows application 
                  "set up USB game controllers". This is a quick and easy way to verify that the masterarm is working
                  correctly. You can click on each channel to then verify that communication between the masterarm and
                  computer has been established.
                  
        Data interpretation;
                
                - During the data transmission process on the masterarm, values are converted to a two byte 
                  integer i.e. a value from 0 to 65534. This process is achieve by linearly scaling each value base on 
                  an expected minimum and maximum. 
                - For position values this mapping is simply 0-2*pi -> 0-65534. Note: this script interprets these
                  the positions in degrees, this can be changed to rad by changing POSITION_SCALE in the core module  
                - For velocity values a saturation is needed. The saturation value was chosen as 10 [rad/s]. 
                
         
    """

    # Create masterarm class and user methods class
    ma = Masterarm()
    um = UserMethods()

    # Set indicators to slow flash
    um.slow_flash_all(ma)
    um.mode = UserMethods.PAUSED

    # Main loop
    while 1:
        # Empty string. Concatenate to this to print to console
        console = ''

        # For every master arm channel
        for ch in ma.channels:

            # Get button state and send new indicator state to masterarm on handle channel
            if ch.channel_idx == Enum.HANDLE:

                # Get the current state of all buttons on this channel (only done here for print to console)
                buttons = um.get_button_state(ch)

                # Add buttons state to console
                console += str(buttons)

                # Set current mode
                if um.toggled_on[Enum.PAUSE] and um.mode == um.PAUSED:
                    um.set_all(ma)
                    um.mode = UserMethods.RUNNING
                elif um.toggled_on[Enum.PAUSE] and um.mode == um.RUNNING:
                    um.slow_flash_all(ma)
                    um.mode = UserMethods.PAUSED

            # When in the running mode, get the joint positions and velocities
            if um.mode == um.RUNNING:
                # Get the current state of all axis on this channel
                axis = um.get_axis_state(ch)

                # Add axis state to console
                console += str([round(a, 3) for a in axis])  # Round axis to 3 decimal places

        sleep(0.1)
        print(console)

