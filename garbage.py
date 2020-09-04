from copy import copy

import pigpio
import sys
from gpiozero import Button
import logging
import argparse
import time
from time import sleep
from signal import pause

logging.basicConfig(format='%(asctime)s %(levelname)-8s %(message)s',
                    level=logging.DEBUG,
                    datefmt='%Y-%m-%d %H:%M:%S')
logger = logging.getLogger(__name__)
logger.propagate = True


class Motor:
    def __init__(self):
        self.pin_direction = None
        self.pin_step = None

    pass


class Switch:
    def __init__(self):
        self.button = None
        self.name = None
        self.position_name = None
        self.pin = None


class Garbage():
    def __init__(self):

        # Declare pins for motor
        # DIR = 12
        # STEP = 18
        self.duty_cycle = 100000  # 50%
        # freq = 1000

        self.crawl_speedfreq = 3000  # Hz
        self.crawl_speed = self.conv_freq_to_mmps(self.crawl_speedfreq)
        self.max_speed = None

        self.motor = Motor()
        self.motor.pin_direction = 12  # Direction GPIO Pin
        self.motor.pin_step = 18  # Step GPIO Pin

        # TODO: Check that pigpiod is running

        # Connect to pigpiod daemon
        self.pi = pigpio.pi()

        # Set up pins as an output
        self.pi.set_mode(self.motor.pin_direction, pigpio.OUTPUT)
        self.pi.set_mode(self.motor.pin_step, pigpio.OUTPUT)

        self.position_names = ['Open', 'ReadyToClose', 'Closed', 'ReadyToOpen']

        # Set switch pins and names
        self.switch_idler_top = Switch()
        self.switch_idler_top.name = 'Idler Top'
        self.switch_idler_top.position_name = self.position_names[0]
        self.switch_idler_top.pin = 19
        self.switch_idler_top.button = Button(self.switch_idler_top.pin, pull_up=False)

        self.switch_idler_bottom = Switch()
        self.switch_idler_bottom.name = 'Idler Bottom'
        self.switch_idler_bottom.position_name = self.position_names[1]
        self.switch_idler_bottom.pin = 13
        self.switch_idler_bottom.button = Button(self.switch_idler_bottom.pin, pull_up=False)

        self.switch_motor_top = Switch()
        self.switch_motor_top.name = 'Motor Top'
        self.switch_motor_top.position_name = self.position_names[3]
        self.switch_motor_top.pin = 6
        self.switch_motor_top.button = Button(self.switch_motor_top.pin, pull_up=False)

        self.switch_motor_bottom = Switch()
        self.switch_motor_bottom.name = 'Motor Bottom'
        self.switch_motor_bottom.position_name = self.position_names[2]
        self.switch_motor_bottom.pin = 5
        self.switch_motor_bottom.button = Button(self.switch_motor_bottom.pin, pull_up=False)

        self.switch_foot = Switch()
        self.switch_foot.name = 'Foot switch'
        self.switch_foot.pin = 27
        self.switch_foot.button = Button(self.switch_foot.pin, pull_up=True)

        # Set switch methods and callbacks
        # Callbacks can't take inputs, so we have to create individual
        # callbacks for each switch.

        self.switch_idler_top.button.when_pressed = self.switch_pressed_idler_top_callback
        self.switch_idler_top.button.when_released = self.switch_released_idler_top_callback

        self.switch_idler_bottom.button.when_pressed = self.switch_pressed_idler_bottom_callback
        self.switch_idler_bottom.button.when_released = self.switch_released_idler_bottom_callback

        self.switch_motor_top.button.when_pressed = self.switch_pressed_motor_top_callback
        self.switch_motor_top.button.when_released = self.switch_released_motor_top_callback

        self.switch_motor_bottom.button.when_pressed = self.switch_pressed_motor_bottom_callback
        self.switch_motor_bottom.button.when_released = self.switch_released_motor_bottom_callback

        self.switch_foot.button.when_pressed = self.switch_pressed_foot_callback

        # set direction to CW
        # motor should only ever rotate CW. CCW rotation just spins
        # the sprag bearing
        # CW = 0 , CCW = 1
        self.rotation_direction = 0

        self.position = None
        self.target_position = None
        self.moving = None

    def switch_pressed_foot_callback(self):
        logger.info('Foot switch triggered')
        if self.position == 'ReadyToClose':
            self.target_position = 'ReadyToOpen'
        elif self.position == 'ReadyToOpen':
            self.target_position = 'ReadyToClose'
        else:
            logger.info(f'Current position [{self.position}] is not'
                        f'appropriate to change target from {self.target_position}')

    def switch_pressed_idler_top_callback(self):
        self.arrived(self.switch_idler_top)

    def switch_released_idler_top_callback(self):
        self.departed(self.switch_idler_top)

    def switch_pressed_idler_bottom_callback(self):
        self.arrived(self.switch_idler_bottom)

    def switch_released_idler_bottom_callback(self):
        self.departed(self.switch_idler_bottom)

    def switch_pressed_motor_top_callback(self):
        self.arrived(self.switch_motor_top)

    def switch_released_motor_top_callback(self):
        self.departed(self.switch_motor_top)

    def switch_pressed_motor_bottom_callback(self):
        self.arrived(self.switch_motor_bottom)

    def switch_released_motor_bottom_callback(self):
        self.departed(self.switch_motor_bottom)

    def arrived(self, data):
        print(f"Hello! Arrived at {data.position_name} position")
        self.position = data.position_name

    def departed(self, data):
        print(f"Goodbye! Departed from {data.position_name}")
        self.last_position = data.position_name
        self.position = None

    def conv_freq_to_mmps(self, data):
        """Converts frequency to mm/s.
        Rough test moved 100mm in 7.5s at 2000Hz, so 100"""

        conversion = (100 / 7.5) / 2000  # Assume 1mm / 1000 Hz
        return data * conversion

    def generate_ramp(self, ramp):
        """Generate ramp wave forms.
        ramp:  List of [Frequency, Steps]
        """
        self.pi.wave_clear()  # clear existing waves
        length = len(ramp)  # number of ramp levels
        wid = [-1] * length

        # Generate a wave per ramp level
        for i in range(length):
            frequency = ramp[i][0]
            micros = int(500000 / frequency)
            wf = []
            wf.append(pigpio.pulse(1 << self.motor.pin_step, 0, micros))  # pulse on
            wf.append(pigpio.pulse(0, 1 << self.motor.pin_step, micros))  # pulse off
            pi.wave_add_generic(wf)
            wid[i] = pi.wave_create()

        # Generate a chain of waves
        chain = []
        for i in range(length):
            steps = ramp[i][1]
            x = steps & 255
            y = steps >> 8
            chain += [255, 0, wid[i], 255, 1, x, y]

        return chain

    def move_to_target(self, target, speed=None):

        duration = 30
        timeout = time.time() + duration

        while self.position != self.target_position:

            if time.time() > timeout:
                raise TimeoutError('Unable to home device after {duration} seconds.')

            if speed != None:
                self.pi.hardware_PWM(self.motor.pin_step, self.crawl_speedfreq, self.duty_cycle)
                self.pi.write(self.motor.pin_direction, self.rotation_direction)  # start rotating
            else:
                self.pi.hardware_PWM(self.motor.pin_step, self.crawl_speedfreq, self.duty_cycle)
                self.pi.write(self.motor.pin_direction, self.rotation_direction)  # start rotating

            sleep(0.1)
        # stop motion
        self.pi.hardware_PWM(self.motor.pin_step, self.crawl_speedfreq, 0)
        logger.info(f'Target position of {self.target_position} reached.')
        self.target_position = None

    def home(self):
        """ This routine crawls until a switch is hit"""

        # Verify that the position isn't already known
        if self.position != None:
            logger.info('Device already homed. Returning')
            return

        self.pi.hardware_PWM(self.motor.pin_step, self.crawl_speedfreq, self.duty_cycle)
        logger.info(f'Finding target, moving with PWM frequency of {self.pi.get_PWM_frequency(self.motor.pin_step)} Hz')

        duration = 30
        timeout = time.time() + duration

        self.pi.write(self.motor.pin_direction, self.rotation_direction)  # start rotating
        while self.position == None:
            if time.time() > timeout:
                raise TimeoutError('Unable to home device after {duration} seconds.')
        # stop motion
        self.pi.hardware_PWM(self.motor.pin_step, self.crawl_speedfreq, 0)
        logger.info('Homing Completed')

    def run(self):
        possible_positions = copy(self.position_names)
        possible_positions.append(None)
        logger.info('Beginning monitor loop')
        try:
            while True:
                # Evaluate if system is happy but not requiring motion
                if self.target_position == None and self.position != None:
                    logger.debug(f'Sleeping for 1s. position is {self.position}')
                    sleep(1)

                # Verify the target name is correct
                if self.target_position not in possible_positions:
                    raise ValueError(f'target position of {self.target_position} is invalid')

                # Door is moved manually
                # potential issue here that door can be moved past two switches
                # not sure this is possible physically, so will not handle yet
                if self.position == None and self.target_position == None:
                    logger.debug('Manual motion detected')
                    if self.last_position == 'Open':
                        logger.warning('Door moved manually from Open position')
                        self.target_position = 'ReadyToClose'
                        self.max_speed = self.crawl_speed
                    elif self.last_position == 'Closed':
                        logger.warning('Door moved manually from Closed position')
                        self.target_position = 'ReadyToOpen'
                        self.max_speed = self.crawl_speed
                    elif self.last_position == 'ReadyToClose':
                        logger.info('Door moved manually from ReadyToClose, closing')
                        self.target_position = 'ReadyToOpen'
                        self.max_speed = self.crawl_speed
                    elif self.last_position == 'ReadyToOpen':
                        logger.info('Door moved manually from ReadyToOpen, opening')
                        self.target_position = 'ReadyToClose'
                        self.max_speed = self.crawl_speed

                # Verify the system is at a ready to open/close state
                if self.position == 'Open':
                    logger.info('Found at Open position, setting target to ReadyToClose')
                    self.target_position = 'ReadyToClose'
                if self.target_position != None:
                    logger.debug(f'Moving to target {self.target_position}')
                    self.move_to_target(self.target_position, speed=self.max_speed)

        except KeyboardInterrupt:
            print("\nCtrl-C pressed.  Stopping PIGPIO and exiting...")
        except Exception as e:
            print(f"Caught Exception in garbage.run() of:  \n {e}")
        finally:
            garbage.pi.set_PWM_dutycycle(garbage.motor.pin_step, 0)  # PWM off
            garbage.pi.stop()
            sys.exit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Monitor and automate garbage motion')

    #    args = parser.parse_args()

    # Instantiate the Garbage Class
    garbage = Garbage()

    # Where are we?
    if garbage.position == None:
        # Home the device by crawling until a switch is hit
        try:
            garbage.home()
        except Exception as e:
            print(f"Caught Exception in garbage.home() of:  \n {e}")
            garbage.pi.set_PWM_dutycycle(garbage.motor.pin_step, 0)  # PWM off
            garbage.pi.stop()
            sys.exit()
        finally:
            print('Homing Complete')

    # Move to a ready-to-open or ready-to-close position
    if garbage.position == ['Open']:
        garbage.target_position == ['Ready to Close']
    elif garbage.position == ['Closed']:
        garbage.target_position == ['Ready to Open']

    # Start the monitor
    print('Starting the monitor')
    garbage.run()

    garbage.pi.set_PWM_dutycycle(garbage.motor.pin_step, 0)  # PWM off
    garbage.pi.stop()
    sys.exit()
