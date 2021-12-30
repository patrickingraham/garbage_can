from time import sleep
import time
import pigpio
import asyncio
import sys

import numpy as np
import time


class testClass:

    def switch_callback(self, gpio, level, tick):
        #print(gpio, level, tick, self.tally_count)
        self.tally_count += 1
        self.tally.append(tick)

        if len(self.tally) > 100:
            time_of_interest = 5  # [s]

            #print(self.tally)
            _arr = np.asarray(self.tally)
            # trim dictionary
            #print(f'array is {_arr}')
            #print(f'time window stars at {tick - time_of_interest*1e6}')

            # indices to be trimmed
            ind = np.where(_arr < (tick - time_of_interest*1e6))[0]

            # so check if there is enough ticks to meet criteria to trigger switch
            if (len(self.tally) - len(ind)):
                print('TRIGGERED SWITCH')
                self.tally = []

            # trim the array of old values
            if len(ind) > 100:
                print(f'trimming tally from {len(self.tally)} to {len(self.tally[-len(ind):])}')
                self.tally = self.tally[-len(ind):]

    def __init__(self):
        print('Inside __init__')

        # Connect to pigpiod daemon
        self.pi = pigpio.pi()

        # Set switch callbacks
        self.switch_idler_top_pin = 27

        ##switch_idler_top = pi.callback(switch_idler_top_pin,
        ##                               pigpio.RISING_EDGE,
        ##                               switch_callback)
        self.switch_idler_top = self.pi.callback(self.switch_idler_top_pin,
                                       pigpio.EITHER_EDGE, self.switch_callback)

        self.tally_count = 0
        self.tally_count0 = 0
        self.tally = []

    def monitor_loop(self):

        print('Starting monitor Loop')
        self.tally_count0 = 5  # when action happens

        try:
            while True:
                #self.tally_count = self.switch_idler_top.tally()
                if self.tally_count > self.tally_count0:
                    print(f'Caught {self.tally_count} edges')

                #self.switch_idler_top.reset_tally()
                self.tally_count = 0
                sleep(0.1)

        except KeyboardInterrupt:
            print("\nCtrl-C pressed.  Stopping PIGPIO and exiting...")
        finally:
            print('done')

if __name__ == "__main__":

    print('inside __main__')
    # execute only if run as a script
    classy = testClass()
    classy.monitor_loop()



def switch_callback(gpio, level, tick):
    print(gpio, level, tick)






### ---


