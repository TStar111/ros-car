"""
#! Python3
Module that mediates python code to interact with Maestro controller
Support for the Pololu Maestro line of servo controllers
Steven Jacobs -- Aug 2013
https://github.com/FRC4564/Maestro/
These functions provide access to many of the Maestro's capabilities using the
Pololu serial protocol
Edited by Dave Foran -- Jan 2020 to fit into Python3 and PEP 8 Conventions
"""
import serial


class Controller():
    """
    When connected via USB, the Maestro creates two virtual serial ports
    /dev/ttyACM0 for commands and /dev/ttyACM1 for communications.
    Be sure the Maestro is configured for "USB Dual Port" serial mode.
    "USB Chained Mode" may work as well, but hasn't been tested.
    Pololu protocol allows for multiple Maestros to be connected to a single
    serial port. Each connected device is then indexed by number.
    This device number defaults to 0x0C (or 12 in decimal), which this module
    assumes.  If two or more controllers are connected to different serial
    ports, or you are using a Windows OS, you can provide the tty port.  For
    example, '/dev/ttyACM2' or for Windows, something like 'COM3'.
    """

    def __init__(self, ttyStr='/dev/ttyACM0', device=0x0c):
        # Open the command port
        self.usb = serial.Serial(ttyStr)
        # Command lead-in and device number are sent for each Pololu serial command.
        self.pololu_cmd = chr(0xaa) + chr(device)
        # Track target position for each servo. The function isMoving() will
        # use the Target vs Current servo position to determine if movement is
        # occuring.  Upto 24 servos on a Maestro, (0-23). Targets start at 0.
        self.targets = [0] * 24
        # Servo minimum and maximum targets can be restricted to protect components.
        self.mins = [0] * 24
        self.maxs = [0] * 24


    def close(self):
        """Cleanup by closing USB serial port"""
        self.usb.close()


    def send_command(self, cmd):
        """Send a Pololu command out the serial port"""
        command_string = self.pololu_cmd + cmd
        self.usb.write(bytes(command_string, 'latin-1'))


    def set_range(self, chan, _min, _max):
        """
        Set channels min and max value range.  Use this as a safety to protect
        from accidentally moving outside known safe parameters. A setting of 0
        allows unrestricted movement.
        ***Note that the Maestro itself is configured to limit the range of servo travel
        which has precedence over these values.  Use the Maestro Control Center to configure
        ranges that are saved to the controller.  Use setRange for software controllable ranges.
        """
        self.mins[chan] = _min
        self.maxs[chan] = _max


    def get_min(self, chan):
        """Return Minimum channel range value"""
        return self.mins[chan]


    def get_max(self, chan):
        """Return Maximum channel range value"""
        return self.maxs[chan]


    def set_target(self, chan, target):
        """
        Set channel to a specified target value.  Servo will begin moving based
        on Speed and Acceleration parameters previously set.
        Target values will be constrained within Min and Max range, if set.
        For servos, target represents the pulse width in of quarter-microseconds
        Servo center is at 1500 microseconds, or 6000 quarter-microseconds
        Typcially valid servo range is 3000 to 9000 quarter-microseconds
        If channel is configured for digital output, values < 6000 = Low ouput
        """
        # if Min is defined and Target is below, force to Min
        if self.mins[chan] > 0 and target < self.mins[chan]:
            target = self.mins[chan]
        # if Max is defined and Target is above, force to Max
        if self.maxs[chan] > 0 and target > self.maxs[chan]:
            target = self.maxs[chan]

        self._set_target(chan, target)


    def _set_target(self, chan, target):
        lsb = target & 0x7f #7 bits for least significant byte
        msb = (target >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.send_command(cmd)
        # Record Target value
        self.targets[chan] = target


    def disable(self, chan):
        self._set_target(chan, 0)


    def set_speed(self, chan, speed):
        """
        Set speed of channel
        Speed is measured as 0.25microseconds/10milliseconds
        For the standard 1ms pulse width change to move a servo between extremes, a speed
        of 1 will take 1 minute, and a speed of 60 would take 1 second.
        Speed of 0 is unrestricted.
        """
        lsb = speed & 0x7f #7 bits for least significant byte
        msb = (speed >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x07) + chr(chan) + chr(lsb) + chr(msb)
        self.send_command(cmd)


    def set_acceleration(self, chan, accel):
        """
        Set acceleration of channel
        This provide soft starts and finishes when servo moves to target position.
        Valid values are from 0 to 255. 0=unrestricted, 1 is slowest start.
        A value of 1 will take the servo about 3s to move between 1ms to 2ms range.
        """
        lsb = accel & 0x7f #7 bits for least significant byte
        msb = (accel >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x09) + chr(chan) + chr(lsb) + chr(msb)
        self.send_command(cmd)


    def get_position(self, chan):
        """
        Get the current position of the device on the specified channel
        The result is returned in a measure of quarter-microseconds, which mirrors
        the Target parameter of setTarget.
        This is not reading the true servo position, but the last target position sent
        to the servo. If the Speed is set to below the top speed of the servo, then
        the position result will align well with the acutal servo position, assuming
        it is not stalled or slowed.
        """
        cmd = chr(0x10) + chr(chan)
        self.send_command(cmd)
        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return (msb << 8) + lsb


    def is_moving(self, chan):
        """
        Test to see if a servo has reached the set target position.  This only provides
        useful results if the Speed parameter is set slower than the maximum speed of
        the servo.  Servo range must be defined first using setRange. See setRange comment.
        ***Note if target position goes outside of Maestro's allowable range for the
        channel, then the target can never be reached, so it will appear to always be
        moving to the target.
        """
        if self.targets[chan] > 0:
            if self.get_position(chan) != self.targets[chan]:
                return True
        return False


    def get_moving_state(self):
        """
        Have all servo outputs reached their targets? This is useful only if Speed and/or
        Acceleration have been set on one or more of the channels. Returns True or False.
        Not available with Micro Maestro.
        """
        cmd = chr(0x13)
        self.send_command(cmd)
        if self.usb.read() == chr(0):
            return False
        else:
            return True


    def run_script_sub(self, sub_number):
        """
        Run a Maestro Script subroutine in the currently active script. Scripts can
        have multiple subroutines, which get numbered sequentially from 0 on up. Code your
        Maestro subroutine to either infinitely loop, or just end (return is not valid).
        """
        cmd = chr(0x27) + chr(sub_number)
        # can pass a param with command 0x28
        # cmd = chr(0x28) + chr(sub_number) + chr(lsb) + chr(msb)
        self.send_command(cmd)


    def stop_script(self):
        """Stop the current Maestro Script"""
        cmd = chr(0x24)
        self.send_command(cmd)

