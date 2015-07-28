#!/usr/bin/env python
import yaml
import sys
import time
import numpy as np
from adc_card_python import *

calibrations = []
card = AdcCard()

WARN_RESISTANCE = 950

print "\nmotor sensor KTY 84-130 calibrator\n"
print "Prepare a ~={} ohm resistor and get a temperature probe\n".format(WARN_RESISTANCE)

def quit():
    card.close()
    sys.exit(0)

def read_channel_mean(chan, sample=50):
    values = []
    for i in range(0, sample):
        values.append(card.read_channel(chan, VoltageRange.U5))
        time.sleep(0.01)
    return np.mean(values)

print "Are you using localGND(0) or userCMMD(1)?"
ans = input("(answer with 0 or 1)\n")
mode = 0
if ans == 0:
    mode = P9116_AI_LocalGND
    print "setting localGND..."
elif ans == 1:
    print "setting userCMMD..."
    mode = P9116_AI_UserCMMD
else:
    print("invalid answer")
    sys.exit(-1)

if not card.open(mode|P9116_AI_UniPolar, 0):
    print("unable to open p9116 card")
    exit(-1)

def ask_next_step():
    raw_input("\nPress enter when done\n")

print ("\nGet a ~={} ohm resistor\n".format(WARN_RESISTANCE))
r_warn_value = input("Insert the actual value of the resistor, measured with"  
        + " a multimeter\n")

min_res=WARN_RESISTANCE-50
max_res=WARN_RESISTANCE+50
if not  min_res <= r_warn_value <= max_res:  
    print "Error: the resistor must be between {} and {}".format(min_res, max_res)
    quit()
for i in range(0, 4):
    print ("\nInsert the ~={} ohm resistor in the motor".format(WARN_RESISTANCE)
            + " temperature sensor connector n. " + str(i+1))
    ask_next_step()
    chan = 13+i
    print "Reading voltage at channel {} ...".format(chan) 
    r_warn_voltage = read_channel_mean(chan)
    print "Voltage at channel {} is {} ".format(chan, r_warn_voltage)
    if not 1.8 <= r_warn_voltage <= 2.2:
        print "Error: the voltage should be ~2.0V"
        print "Is the resistor connected? Is the card configuration ok?"
        quit()
    print "Ok, now connect the KTY 84-130 resistor of motor {}".format(i+1)
    ask_next_step()
    print "Reading voltage at channel {} ...".format(chan)
    normal_voltage = read_channel_mean(chan)
    print "Voltage is {}".format(normal_voltage)
    print "Get the current temperature of the motor"
    print "it must not be powered, it should have the ambient temperature\n"
    normal_temp = input("insert the temperature value (Celsius)\n")
    new_cal = {
            "temp_sensor_calibration_motor_" + str(i) : {
                "base_temp_v" : float(normal_voltage),
                "danger_v" : float(r_warn_voltage),
                "base_temp" : float(normal_temp),
                "temp_volt_diff" : float ( (100 - normal_temp) / (normal_voltage - r_warn_voltage ) )
                }
            }
    print "Done motor {}\n\n".format(i+1)
    calibrations.append(new_cal)


with open('temp_sensor_calibration_motor.yaml', 'w') as outfile:
    text = ""
    for cal in calibrations:
        text += yaml.dump(cal, default_flow_style=False) + "\n"
    print text
    outfile.write(text)
