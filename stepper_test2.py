from time import sleep
import time
import pigpio
import asyncio
import sys

def generate_ramp(ramp):
    """Generate ramp wave forms.
    ramp:  List of [Frequency, Steps]
    """
    pi.wave_clear()     # clear existing waves
    length = len(ramp)  # number of ramp levels
    wid = [-1] * length

    # Generate a wave per ramp level
    for i in range(length):
        frequency = ramp[i][0]
        micros = int(500000 / frequency)
        wf = []
        wf.append(pigpio.pulse(1 << STEP, 0, micros))  # pulse on
        wf.append(pigpio.pulse(0, 1 << STEP, micros))  # pulse off
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

def switch_callback(gpio, level, tick):
    print(gpio, level, tick)


DIR = 12     # Direction GPIO Pin
STEP = 18    # Step GPIO Pin

# Connect to pigpiod daemon
pi = pigpio.pi()

# Set up pins as an output
pi.set_mode(DIR, pigpio.OUTPUT)
pi.set_mode(STEP, pigpio.OUTPUT)

# Set switch callbacks
switch_idler_top_pin = 19
switch_idler_lower_pin = 13
switch_motor_top_pin = 6
switch_motor_lower_pin = 5
##switch_idler_top = pi.callback(switch_idler_top_pin,
##                               pigpio.RISING_EDGE,
##                               switch_callback)
switch_idler_top = pi.callback(switch_idler_top_pin,
                               pigpio.RISING_EDGE) 
pi.set_mode(switch_idler_top_pin, pigpio.INPUT)
pi.set_pull_up_down(switch_idler_top_pin, pigpio.PUD_DOWN)



# set direction to CW
# motor should only ever rotate CW.
CW = 0 ; CCW = 1 
#pi.set_pull_up_down(DIR, 0)
#pi.write(DIR, CW) # start rotating 


if True:
    print('starting test loop')
    #pi.set_PWM_frequency(STEP, 160000)  # 500 pulses per second
    freq = 2000
    duty_cycle= 100000 # 50%
    pi.hardware_PWM(STEP, freq, duty_cycle)
    print(f'PWM frequency is {pi.get_PWM_frequency(STEP)} Hz')
    #pi.set_PWM_dutycycle(STEP, 128)  # PWM 1/2 On 1/2 Off
    try:
        tmp=0
        while True:
            pi.write(DIR, CW) # start rotating
            #print(pi.read(switch_idler_top_pin))
            if tmp != switch_idler_top.tally():
                print(f'tally is {tmp}')
                tmp=switch_idler_top.tally()
            sleep(0.1)

    except KeyboardInterrupt:
        print("\nCtrl-C pressed.  Stopping PIGPIO and exiting...")
        print(f'Tally was {switch_idler_top.tally()}')
    finally:
        pi.set_PWM_dutycycle(STEP, 0)  # PWM off
        pi.set_PWM_dutycycle(STEP, 0)  # PWM off
        pi.stop()
        sys.exit()

print(f'asdfasd {asdf}')
# Ramp up
chain = generate_ramp([[320, 200],
	       [500, 400],
	       [800, 500],
	       [1000, 700],
	       [1600, 900],
	       [2000, 10000]])

#print(f'PWM frequency is {pi.get_PWM_frequency(STEP)} Hz') # 500 by default

# Command returns immediately!
pi.wave_chain(chain)  # Transmit chain 
sleep(2)

if True:
    pi.wave_tx_stop()
    print('Stopped chain')

# Need to understand how to know when chain is completed
# and how to wait for completion

#print('Chain Completed')
print(f'PWM frequency is {pi.get_PWM_frequency(STEP)} Hz')

# Set duty cycle and frequency
pi.set_PWM_dutycycle(STEP, 128)  # PWM 1/2 On 1/2 Off
#pi.set_PWM_dutycycle(STEP, 255)  # Always on? or always off?
pi.set_PWM_frequency(STEP, 500)  # 500 pulses per second
sleep(1)
print('Entering While Loop, Cntl+C to exit')

freq = pi.get_PWM_frequency(STEP)
try:
    while True:
        #pi.write(DIR, pi.read(SWITCH))  # Set direction
        #pi.write(DIR, 0) # start rotating
        
        #pi.write(DIR, CW) # start rotating
        ramp_def = [[320, 20], [500, 40],[800, 50], [1000, 70],
	       [1600, 90],
	       [2000, 1000]] 
        
        chain = generate_ramp(ramp_def)
        pi.wave_chain(chain)  # Transmit chain 
        
        steps=0
        for speeds in ramp_def:
            steps += speeds[1] 
    
        print(f'total steps is {steps}')
        
        sleep(steps/freq/2)

except KeyboardInterrupt:
    print ("\nCtrl-C pressed.  Stopping PIGPIO and exiting...")
finally:
    pi.set_PWM_dutycycle(STEP, 0)  # PWM off
    pi.stop()