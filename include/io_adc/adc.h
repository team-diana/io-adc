// Software License Agreement (BSD License)
//
// Copyright (c) 2013, Tamer Saadeh
// <tamer@tamersaadeh.com>
// based on work by Mattia Marenco
// <mattia.marenco@teamdiana.org>
//
// All rights reserved.

#ifndef _ADC_H
#define _ADC_H

#define Chan_IMU 6
#define Chan_sosp 8
#define Chan_range 4
#define Chan_imu_2 3
#define Chan_diag 3

#define RANGE_TWO 5
#define RANGE_THREE 2.5
#define RANGE_FOUR 1.25

// For the accelerometer and gyroscope constants
#define AG_MAX_VALUE 32768
#define AG_REF_VOLTAGE 5
#define AG_ZERO_RATE_VOLTAGE CALIBRATION * AG_REF_VOLTAGE / AG_MAX_VALUE

// Calibration constants
#define CALIBRATION 140
#define CALIB_ELEMENTS 17

// Value of g
#define G_CONSTANT 9.7803084

// TODO these values need calibration
#define IMU1_G 0.300
#define IMU2_G 0.800

// Converts from rad to degrees
#define RAD_TO_DEG(x) (x / (2 * M_PI / 360.0))

// Set the max range given a boolean expression, the switch case condition, and the variable to modify
#define SET_MAX_RANGE(bool_exp, switch_case, var) ( \
if (bool_exp) { \
switch (switch_case) { \
case(2): \
var = RANGE_TWO; \
break; \
case(3): \
var = RANGE_THREE; \
break; \
case(4): \
var = RANGE_FOUR; \
break; \
} \
} \
)

// Update a variable and publish it's status, given an index and imu
#define UPDATE_AXIS(m, calibration, index, imu) ( \
if (filter_imu) { \
#m.push_back(float(cdata)); \
BOOST_FOREACH (float reading, #m) \
{ \
total += reading; \
} \
cdata = total / #m.size(); \
} \
updateImuState(#imu, cdata, #index, #calibration); \
)

// Updates the values of the pitch
#define PITCH_UPDATE(pitch) ( \
#pitch.push_back(float(pitch)); \
BOOST_FOREACH(float reading, #pitch) \
{ \
total += reading; \
} \
sosp_data[i / 2] = total / #pitch.size(); \
)

// Updates the ranges
#define UPDATE_RANGE(range) ( \
#range.push_back(float(chan_voltage[i])); \
BOOST_FOREACH(float reading, #range) \
{ \
total += reading; \
} \
cdata = total / #range.size(); \
)

// limit the output of the range finder
#define LIMIT_RANGE() ( \
if (range_temp > range.max_range) \
range.range = range.max_range; \
else if (range_temp < range.min_range) \
range.range = range.min_range; \
else \
range.range = range_temp; \
)

// Sampling of the filtered data
// defines the amount of samples to use before averaging the filtered result
#define SAMPLE_FILTERED_EVERY 10
#define SAMPLE_FILTERED_EVERY_RANGE 2
#define SAMPLE_SIZE_50 50
#define SAMPLE_SIZE_10 10
#define SAMPLE_SIZE_5 5
#define RATE 100

// Range constants
#define CYCLE_RANGE 90

// Initialization timer length
#define INITIALIZATION_TIMER 1000

// for IMU
bool isMoving = false;
bool startup = true;
double calibrationOffset[17] = {140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140};
double rover_x, rover_y, rover_z;

#endif
