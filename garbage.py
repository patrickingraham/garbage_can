from copy import copy

import pigpio
import sys
from gpiozero import Button
import logging
import argparse
import time
from time import sleep
import numpy as np
from signal import pause

logging.basicConfig(
    format="%(asctime)s %(levelname)-8s %(message)s",
    level=logging.DEBUG,
    datefmt="%Y-%m-%d %H:%M:%S",
)
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


class Garbage:
    def __init__(self):
        # TODO: Check that pigpiod is running

        # Connect to pigpiod daemon
        self.pi = pigpio.pi()

        # Declare pins for motor
        self.duty_cycle = 100000  # 50%

        self.motor = Motor()
        self.motor.pin_direction = 12  # Direction GPIO Pin
        self.motor.pin_step = 18  # Step GPIO Pin

        # Set up pins as an output
        self.pi.set_mode(self.motor.pin_direction, pigpio.OUTPUT)
        self.pi.set_mode(self.motor.pin_step, pigpio.OUTPUT)

        self.position_names = ["Open", "ReadyToClose", "Closed", "ReadyToOpen"]

        # Set switch pins and names
        self.switch_idler_top = Switch()
        self.switch_idler_top.name = "Idler Top"
        self.switch_idler_top.position_name = self.position_names[0]
        self.switch_idler_top.pin = 19
        self.switch_idler_top.button = Button(self.switch_idler_top.pin, pull_up=False)

        self.switch_idler_bottom = Switch()
        self.switch_idler_bottom.name = "Idler Bottom"
        self.switch_idler_bottom.position_name = self.position_names[1]
        self.switch_idler_bottom.pin = 13
        self.switch_idler_bottom.button = Button(
            self.switch_idler_bottom.pin, pull_up=False
        )

        self.switch_motor_top = Switch()
        self.switch_motor_top.name = "Motor Top"
        self.switch_motor_top.position_name = self.position_names[3]
        self.switch_motor_top.pin = 6
        self.switch_motor_top.button = Button(self.switch_motor_top.pin, pull_up=False)

        self.switch_motor_bottom = Switch()
        self.switch_motor_bottom.name = "Motor Bottom"
        self.switch_motor_bottom.position_name = self.position_names[2]
        self.switch_motor_bottom.pin = 5
        self.switch_motor_bottom.button = Button(
            self.switch_motor_bottom.pin, pull_up=False
        )

        self.switch_foot = Switch()
        self.switch_foot.name = "Foot switch"
        self.switch_foot.pin = 27
        self.switch_foot.button = Button(self.switch_foot.pin, pull_up=True)

        # Set switch methods and callbacks
        # Callbacks can't take inputs, so we have to create individual
        # callbacks for each switch.

        self.switch_idler_top.button.when_pressed = (
            self.switch_pressed_idler_top_callback
        )
        self.switch_idler_top.button.when_released = (
            self.switch_released_idler_top_callback
        )

        self.switch_idler_bottom.button.when_pressed = (
            self.switch_pressed_idler_bottom_callback
        )
        self.switch_idler_bottom.button.when_released = (
            self.switch_released_idler_bottom_callback
        )

        self.switch_motor_top.button.when_pressed = (
            self.switch_pressed_motor_top_callback
        )
        self.switch_motor_top.button.when_released = (
            self.switch_released_motor_top_callback
        )

        self.switch_motor_bottom.button.when_pressed = (
            self.switch_pressed_motor_bottom_callback
        )
        self.switch_motor_bottom.button.when_released = (
            self.switch_released_motor_bottom_callback
        )

        self.switch_foot.button.when_pressed = self.switch_pressed_foot_callback

        # set direction to CW
        # motor should only ever rotate CW. CCW rotation just spins
        # the sprag bearing
        # CW = 0 , CCW = 1
        self.rotation_direction = 0


        self.pulse_per_motor_rev = 1600
        self.gearbox_ratio = 20
        self.dist_per_rev = 2 * np.pi * 79.070/2.0  # 496.8 [mm]
        self.steps_per_mm = (self.gearbox_ratio * self.pulse_per_motor_rev) / self.dist_per_rev # 128.8
        logger.debug(f'motor will do {self.steps_per_mm} steps per mm')
        self.crawl_speedfreq = 3000  # Hz
        self.freq_to_mmps = self.steps_per_mm
        #conversion = (100 / 7.5) / 2000  # Assume 1mm / 1000 Hz

        self.crawl_speed = self.crawl_speedfreq / self.steps_per_mm  # 12.5 mm/s
        self.separation_distance = 300  # [mm]
        _reduction_factor = 0.5
        self.max_speed = 300 * _reduction_factor  # 300 mm/s ~ 45000 Hz
        self.acceleration = 300 * _reduction_factor  # mm/s2
        # self.steps_per_mm = 150  # 150 steps/mm

        # time to accelerate
        _accel_time = self.max_speed / self.acceleration  # [s]
        # distance covered in accelleration d= Vi*t + 0.5 a * t^2
        _accel_dist_steps = (0.5 * self.acceleration * _accel_time**2) * self.steps_per_mm  #
        # So need a ramp that goes from 0 to self.max_speed in_accel_dist_steps
        # intervals need to be defined as frequencies
        # assume a smooth ramp
        _number_of_intervals = 10
        logger.debug(f'acceleration ramp starting speed is {self.max_speed*self.steps_per_mm / _number_of_intervals}')
        logger.debug(f'max acceleration ramp speed iis {self.max_speed*self.steps_per_mm}')
        accel_intervals = np.arange(self.max_speed*self.steps_per_mm / _number_of_intervals,
                                    self.max_speed*self.steps_per_mm,
                                    self.max_speed*self.steps_per_mm / _number_of_intervals, dtype=np.int32)
        accel_steps_per_interval = int(_accel_dist_steps / _number_of_intervals)
        # Assume deceleration is the reverse of acceleration
        deccel_intervals = np.arange(self.max_speed*self.steps_per_mm,
                                     self.crawl_speed*self.steps_per_mm,
                                     (self.crawl_speed-self.max_speed) * self.steps_per_mm / _number_of_intervals,
                                     dtype=np.int32)

        ramp_list = []
        _interval = 0
        _steps = 0.0
        _deceleration_start = self.separation_distance*self.steps_per_mm - _accel_dist_steps
        _steps_at_max_speed = self.separation_distance * self.steps_per_mm - 2*_accel_dist_steps

        # steps=0
        # while steps < self.separation_distance*self.steps_per_mm:
        #     if steps < _accel_dist_steps:
        #         ramp_list.append([accel_intervals[_interval], accel_steps_per_interval])
        #         steps+=
        #     if (steps >= _accel_dist_steps) and (steps <= _decelleration_start):

        # Create ramp
        # Create acceleration
        for i in accel_intervals:
            ramp_list.append([i, accel_steps_per_interval])

        logger.debug(f'ramp list after acceleration is {ramp_list}')
        # Section at max speed
        ramp_list.append([np.round(self.max_speed * self.steps_per_mm), accel_steps_per_interval])
        logger.debug(f'ramp list after max_speed is {ramp_list}')
        # Decellerate
        for i in deccel_intervals:
            ramp_list.append([i, accel_steps_per_interval])
        logger.debug(f'ramp list after deceleration is {ramp_list}')

        # Generate the wavechain
        self.open_ramp = self.generate_ramp(ramp_list)

        # Variables used in control loops
        self.position = None
        self.target_position = None
        self.moving = None

    def switch_pressed_foot_callback(self):
        logger.info("Foot switch triggered")
        if self.position == "ReadyToClose":
            self.target_position = "ReadyToOpen"
        elif self.position == "ReadyToOpen":
            self.target_position = "ReadyToClose"
        else:
            logger.info(
                f"Current position [{self.position}] is not"
                f"appropriate to change target from {self.target_position}"
            )

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

    def generate_ramp(self, ramp):
        """Generate ramp wave forms.
        ramp:  List of [Frequency, Steps]
        """
        logger.debug(f'Generating ramp from {ramp}')

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
            self.pi.wave_add_generic(wf)
            wid[i] = self.pi.wave_create()

        # Generate a chain of waves
        chain = []
        for i in range(length):
            steps = ramp[i][1]
            x = steps & 255
            y = steps >> 8
            chain += [255, 0, wid[i], 255, 1, x, y]

        return chain

    def move_to_target(self, speed=None):

        duration = 30
        timeout = time.time() + duration

        if speed == None:
            speed=self.crawl_speed

        logger.debug(f'Starting move to target {self.target_position} at speed {speed}')

        while self.position != self.target_position:

            if time.time() > timeout:
                raise TimeoutError("Unable to move device to target after {duration} seconds.")

            if speed == self.max_speed:

                if not self.pi.wave_tx_busy():
                    self.pi.wave_chain(self.open_ramp)  # Transmit chain
                # while pi.wave_tx_busy():  # wait for waveform to be sent
                #     time.sleep(0.02)

            else:
                self.pi.hardware_PWM(
                    self.motor.pin_step, self.crawl_speedfreq, self.duty_cycle
                )
                self.pi.write(
                    self.motor.pin_direction, self.rotation_direction
                )  # start rotating

            sleep(0.1)
        # stop motion
        self.pi.hardware_PWM(self.motor.pin_step, self.crawl_speedfreq, 0)
        logger.info(f"Target position of {self.target_position} reached.")
        self.target_position = None

    def home(self):
        """ This routine crawls until a switch is hit"""

        # Verify that the position isn't already known
        if self.position != None:
            logger.info("Device already homed. Returning")
            return

        self.pi.hardware_PWM(self.motor.pin_step, self.crawl_speedfreq, self.duty_cycle)
        logger.info(
            f"Finding target, moving with PWM frequency of {self.pi.get_PWM_frequency(self.motor.pin_step)} Hz"
        )

        duration = 30
        timeout = time.time() + duration

        self.pi.write(
            self.motor.pin_direction, self.rotation_direction
        )  # start rotating
        while self.position == None:
            if time.time() > timeout:
                raise TimeoutError("Unable to home device after {duration} seconds.")
        # stop motion
        self.pi.hardware_PWM(self.motor.pin_step, self.crawl_speedfreq, 0)
        logger.info("Homing Completed")

    def run(self):
        possible_positions = copy(self.position_names)
        possible_positions.append(None)
        logger.info("Beginning monitor loop")
        try:
            while True:
                # Evaluate if system is happy but not requiring motion
                if self.target_position == None and self.position != None:
                    logger.debug(f"Sleeping for 1s. position is {self.position}")
                    sleep(1)

                # Verify the target name is correct
                if self.target_position not in possible_positions:
                    raise ValueError(
                        f"target position of {self.target_position} is invalid"
                    )

                # Door is moved manually
                # potential issue here that door can be moved past two switches
                # not sure this is possible physically, so will not handle yet
                if self.position == None and self.target_position == None:
                    logger.debug("Manual motion detected")
                    if self.last_position == "Open":
                        logger.warning("Door moved manually from Open position")
                        self.target_position = "ReadyToClose"
                        self.max_speed = self.crawl_speed
                    elif self.last_position == "Closed":
                        logger.warning("Door moved manually from Closed position")
                        self.target_position = "ReadyToOpen"
                        self.max_speed = self.crawl_speed
                    elif self.last_position == "ReadyToClose":
                        logger.info("Door moved manually from ReadyToClose, closing")
                        self.target_position = "ReadyToOpen"
                        self.max_speed = self.crawl_speed
                    elif self.last_position == "ReadyToOpen":
                        logger.info("Door moved manually from ReadyToOpen, opening")
                        self.target_position = "ReadyToClose"
                        self.max_speed = self.crawl_speed

                # Verify the system is at a ready to open/close state
                if self.position == "Open":
                    logger.info(
                        "Found at Open position, setting target to ReadyToClose"
                    )
                    self.target_position = "ReadyToClose"
                if self.position == "Closed":
                    logger.info(
                        "Found at Closed position, setting target to ReadyToOpen"
                    )
                    self.target_position = "ReadyToClose"
                if self.target_position != None:
                    logger.debug(f"Monitor sending command to move to target {self.target_position} "
                                 f"at speed {self.max_speed}")
                    self.move_to_target(speed=self.max_speed)

        except KeyboardInterrupt:
            print("\nCtrl-C pressed.  Stopping PIGPIO and exiting...")
        except Exception as e:
            print(f"Caught Exception in garbage.run() of:  \n {e}")
        finally:
            garbage.pi.set_PWM_dutycycle(garbage.motor.pin_step, 0)  # PWM off
            garbage.pi.stop()
            sys.exit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor and automate garbage motion")

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
            print("Homing Complete")

    # Move to a ready-to-open or ready-to-close position
    if garbage.position == ["Open"]:
        garbage.target_position == ["Ready to Close"]
    elif garbage.position == ["Closed"]:
        garbage.target_position == ["Ready to Open"]

    # Start the monitor
    print("Starting the monitor")
    garbage.run()

    garbage.pi.set_PWM_dutycycle(garbage.motor.pin_step, 0)  # PWM off
    garbage.pi.stop()
    sys.exit()
