// Software License Agreement (BSD License)
//
// Copyright (c) 2013, Mattia Marenco
// <mattia.marenco@teamdiana.org>
// All rights reserved.

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include "adc/imu_Adc.h"
#include "adc/sosp_Adc.h"
#include "adc/diag_Adc.h"
#include "adc/movingService.h"
#include "tf/transform_broadcaster.h"
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include "dask.h"
#include "conio.h"
#include "adc.h"

/**
 *
 *  Update the IMU message given a buffer and raw data???
 *
 *  Parameters:
 *      imuMsg: the IMU message to update
 *      chan_data_raw: raw channel data to ???
 *      i: index of what?!?!?!?!?!
 *      calibration: some buffer of calibration but not the raw???
 *
 *  Returns:
 *      nothing
 */

void updateImuState(sensor_msgs::Imu& imuMsg, I16 chan_data_raw, int i, boost::circular_buffer<float>& calibration)
{
    double current_time = ros::Time::now().toSec();
    double last_time = imuMsg.header.stamp.toSec();
    double linear_acceleration;
    double angular_velocity;
    double g;

    if (((!isMoving) && (i < 3)) || (startup)) {
        calibration.push_back(float(chan_data_raw));
        double total = 0;

        BOOST_FOREACH (float reading, calibration) {
            total += reading;
        }
        calibrationOffset[i] = total / calibration.size();
        ROS_INFO("Calibration channel %d", i);
    }

    // accelerometer
    if (i >= 3) {
        double zeroRateV = AG_ZERO_RATE_VOLTAGE;
        double sensitivity;

        //IMU1
        if (i == 5) {
            zeroRateV -= IMU1_G;
        }

        //su asse z aggiungo la g
        if (i <= 5) {
            sensitivity = IMU1_G / G_CONSTANT; //300mv/g  [V/ m/s^2]
        }

        //IMU2
        if (i == 8) {
            zeroRateV -= IMU2_G;
        }

        //su asse z aggiungo la g
        if ((i > 5) && (i <= 8)) {
            sensitivity = IMU2_G / G_CONSTANT; //800mv/g  [V/ m/s^2]
        }

        linear_acceleration = (chan_data_raw * AG_REF_VOLTAGE / AG_MAX_VALUE - zeroRateV) / sensitivity;
    }

    // gyroscope
    if (i < 3) {
        double sensitivity = RAD_TO_DEG(0.00333); //3.33mv/�/s  [V/rad/s]
        angular_velocity = (chan_data_raw * AG_REF_VOLTAGE / AG_MAX_VALUE - AG_ZERO_RATE_VOLTAGE) / sensitivity;
    }

    // add time to the message header
    imuMsg.header.stamp = ros::Time::now();
    switch (i) {
            // Angulare velocity
            // x
        case(0):
            imuMsg.angular_velocity.y = angular_velocity;
            break;
            // y
        case(1):
            imuMsg.angular_velocity.x = -angular_velocity;
            break;
            // z
        case(2):
            imuMsg.angular_velocity.z = angular_velocity;
            break;
            // Linear acceleration
            // FIXME: original code had the variables x and y flipped for linear_acceleration, why?
            // x
        case(3):
            rover_x = linear_acceleration;
        case(6):
            imuMsg.linear_acceleration.x = linear_acceleration;
            break;
            // y
        case(4):
            rover_y = linear_acceleration;
        case(7):
            imuMsg.linear_acceleration.y = -linear_acceleration;
            break;
            // z
        case(5):
            rover_z = linear_acceleration;
        case(8):
            imuMsg.linear_acceleration.z = linear_acceleration;
            break;
    }
}

/**
 *
 *  Sets the /moving ROS topic values of the rover
 *
 *  Parameters:
 *      req: service request??
 *      res: service respone??
 *
 *  Returns:
 *      bool: true for successful publishing and fails otherwise
 */

bool movingStatus(adc::movingService::Request &req, adc::movingService::Response &res)
{
    isMoving = req.status;
    ros::param::set("/moving", isMoving);
    return true;
}

/**
 *
 *  Main function. ROS set up
 *
 *  Parameters:
 *      argc: number of command-line parameters
 *      argv: command-line parameters
 *
 *  Returns:
 *      int: exit status
 */

// FIXME: look into the strange loops/indicies jumping aroung for possible optimizations
int main(int argc, char **argv)
{
    // Set up ROS
    std::cout << "start" << std::endl;
    ros::init(argc, argv, "adc");
    ros::NodeHandle n;

    // IMU message
    sensor_msgs::Imu imu,imu_2;
    sensor_msgs::Range range;

    // What is this?! this is the declaration of all the circular buffer for calibration and mean
    boost::circular_buffer<float> calibration0(CALIBRATION),
    calibration1(CALIBRATION),
    calibration2(CALIBRATION),
    calibration3(CALIBRATION),
    calibration4(CALIBRATION),
    calibration5(CALIBRATION),
    calibration6(CALIBRATION),
    calibration7(CALIBRATION),
    calibration8(CALIBRATION),
    range1(SAMPLE_SIZE_5),
    range2(SAMPLE_SIZE_5),
    range3(SAMPLE_SIZE_5),
    range4(SAMPLE_SIZE_5),
    pitch1(SAMPLE_SIZE_50),
    pitch2(SAMPLE_SIZE_50),
    pitch3(SAMPLE_SIZE_50),
    pitch4(SAMPLE_SIZE_50),
    x(SAMPLE_SIZE_50),
    y(SAMPLE_SIZE_50),
    z(SAMPLE_SIZE_50),
    ax(SAMPLE_SIZE_10),
    ay(SAMPLE_SIZE_10),
    az(SAMPLE_SIZE_10),
    ax2(SAMPLE_SIZE_10),
    ay2(SAMPLE_SIZE_10),
    az2(SAMPLE_SIZE_10),
    calib_sosp0(CALIBRATION),
    calib_sosp1(CALIBRATION),
    calib_sosp2(CALIBRATION),
    calib_sosp3(CALIBRATION),
    calib_sosp4(CALIBRATION),
    calib_sosp5(CALIBRATION),
    calib_sosp6(CALIBRATION),
    calib_sosp7(CALIBRATION);

    // Declare variables that can be modified by launch file.
    std::string message;
    int rate;

    std::string topic_imu,
    topic_sosp,
    topic_rangef,
    topic_rangefd,
    topic_rangep,
    topic_rangepd,
    topic_imu_2,
    topic_diag;

    int range_imu,
    range_sosp,
    range_range,
    range_imu_2,
    range_diag;

    bool enable_imu,
    enable_sosp,
    enable_range,
    filter_imu,
    enable_imu_2,
    enable_diag;

    int i;
    char temp_char;

    I16 card = -1;
    I16 err;
    I16 dio = -1;

    // TODO configurare meglio il massimo del buffer
    I16 chan_data[8];
    I16 chan_data_raw[8];
    F32 sosp_data[4];
    F32 chan_voltage[8];
    F32 max_voltage;
    F32 max_voltage_range;
    F32 max_voltage_diag;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can
    // be run simultaneously while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("message", message, std::string("NO error"));
    private_node_handle_.param("rate", rate, int(RATE));
    private_node_handle_.param("topic_imu", topic_imu, std::string("ADC/IMU"));
    private_node_handle_.param("range_imu", range_imu, int(3));
    private_node_handle_.param("enable_imu", enable_imu, bool(true));
    private_node_handle_.param("topic_sosp", topic_sosp, std::string("ADC/suspension"));
    private_node_handle_.param("range_sosp", range_sosp, int(3));
    private_node_handle_.param("enable_sosp", enable_sosp, bool(true));
    private_node_handle_.param("topic_range", topic_rangef, std::string("ADC/range_front"));
    private_node_handle_.param("topic_range", topic_rangefd, std::string("ADC/range_front_down"));
    private_node_handle_.param("topic_range", topic_rangep, std::string("ADC/range_post"));
    private_node_handle_.param("topic_range", topic_rangepd, std::string("ADC/range_post_down"));
    private_node_handle_.param("range_range", range_range, int(2));
    private_node_handle_.param("enable_range", enable_range, bool(true));
    private_node_handle_.param("topic_imu_2", topic_imu_2, std::string("ADC/IMU_2"));
    private_node_handle_.param("range_imu_2", range_imu_2, int(3));
    private_node_handle_.param("enable_imu_2", enable_imu_2, bool(false));
    private_node_handle_.param("topic_diag", topic_diag, std::string("ADC/diag"));
    private_node_handle_.param("range_diag", range_diag, int(2));
    private_node_handle_.param("enable_diag", enable_diag, bool(true));

    n.setParam("/moving", bool(false));
    n.setParam("/adc/filtered_imu", bool(false));

    if ((card = Register_Card(PCI_9116, 0)) < 0) {
        std::cout <<  "ADC Register_Card error= " <<  card << std::endl;
        message = "ADC Register_Card error";
        exit(1);
    }
    
    if ((*err = AI_9116_Config(card, P9116_AI_SingEnded | P9116_AI_UserCMMD, P9116_AI_SoftPolling, 0, 0, 0)) != NoError) {
        printf("Config error : error_code: %d \n", err);
        std::stringstream ms;
        ms << "Config error Error: error_code: " << int(err);
        message = ms.str();
    }

    if ((dio = Register_Card(PCI_7432, 0)) < 0) {
        std::cout <<  "DIO Register_Card error= " <<  dio << std::endl;
        message = "DIO Register_Card error";
        exit(1);
    }

    SET_MAX_RANGE(enable_imu || enable_sosp || enable_imu_2, range_imu, max_voltage);
    SET_MAX_RANGE(enable_range, range_range, max_voltage_range);
    SET_MAX_RANGE(enable_diag, range_diag, max_voltage_diag);

/*
     
     // This should be implemented in the macro SET_MAX_RANGE
     
     if (enable_imu || enable_sosp || enable_imu_2) {
     switch (range_imu) {
     case(2):
     max_voltage = RANGE_TWO;
     break;
     case(3):
     max_voltage = RANGE_THREE;
     break;
     case(4):
     max_voltage = RANGE_FOUR;
     break;
     }
     //TODO dare un colpo di HP reset ad ogni accensione tramite DIO
     //DO_WriteLine(dio, 0,0, 1);
     //usleep(25000000);
     //DO_WriteLine(dio, 0,0, 0);
     }
     
     if (enable_range) {
     switch (range_range) {
     case(2):
     max_voltage_range = RANGE_TWO;
     break;
     case(3):
     max_voltage_range = RANGE_THREE;
     break;
     case(4):
     max_voltage_range = RANGE_FOUR;
     break;
     }
     }
     
     if (enable_diag) {
     switch (range_diag) {
     case(2):
     max_voltage_diag = RANGE_TWO;
     break;
     case(3):
     max_voltage_diag = RANGE_THREE;
     break;
     case(4):
     max_voltage_diag = RANGE_FOUR;
     break;
     }
     }
     
     */

    // FIXME: add documentation to the following overall code functionality
    // TODO: config; fill in IMU data (start at 0 orientation)
    imu.orientation = tf::createQuaternionMsgFromYaw(0.0);
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "chassis";
    boost::array<double, 9> cov = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}};
    boost::array<double, 9> cov2 = {{-1, 0, 0, 0, 0, 0, 0, 0, 0}};
    std::copy(cov.begin(), cov.end(), imu.orientation_covariance.begin());
    std::copy(cov.begin(), cov.end(), imu.angular_velocity_covariance.begin());
    std::copy(cov2.begin(), cov2.end(), imu.linear_acceleration_covariance.begin());

    ros::Publisher imu_adc_pub = n.advertise<adc::imu_Adc>(topic_imu.c_str(), 100);
    ros::Publisher imu_raw_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
    ros::Publisher sosp_adc_pub = n.advertise<adc::sosp_Adc>(topic_sosp.c_str(), 100);
    ros::Publisher rangef_pub = n.advertise<sensor_msgs::Range>(topic_rangef.c_str(), 100);
    ros::Publisher rangefd_pub = n.advertise<sensor_msgs::Range>(topic_rangefd.c_str(), 100);
    ros::Publisher rangep_pub = n.advertise<sensor_msgs::Range>(topic_rangep.c_str(), 100);
    ros::Publisher rangepd_pub = n.advertise<sensor_msgs::Range>(topic_rangepd.c_str(), 100);
    ros::Publisher imu_2_adc_pub = n.advertise<adc::imu_Adc>(topic_imu_2.c_str(), 100);
    ros::Publisher imu_2_raw_pub = n.advertise<sensor_msgs::Imu>("imu_2/data_raw", 100);
    ros::Publisher diag_adc_pub = n.advertise<adc::diag_Adc>(topic_diag.c_str(), 100);
    ros::ServiceServer service = n.advertiseService("Moving_Status", movingStatus);
    ROS_INFO("Ready to go.");

    // Tell ROS how fast to run this node.
    ros::Rate loop_rate(rate);

    // Main loop.
    long int count = 0;
    int count_filtered_imu = 0;
    int count_range = 0;
    while (n.ok()) {
        bool filter_imu_temp = filter_imu;
        if (!n.getParam("/moving", isMoving)) {
            isMoving = false;
            n.setParam("/moving", bool(false));
        }

        if (!n.getParam("/adc/filtered_imu", filter_imu)) {
            filter_imu = false;
            n.setParam("/adc/filtered_imu", bool(false));
            if ((filter_imu_temp != filter_imu) && (filter_imu))
                count = 0;
        }
        
        if ((enable_imu) && (count_range < CICLE_RANGE)) {
            for (int i = 0; i < Chan_IMU; i++) {
                if ((err = AI_ReadChannel(card, i, range_imu, (U16*) &chan_data_raw[i])) != NoError) {
                    printf("AI_ReadChannel Ch#%d error : error_code: %d \n", i, err);
                    std::stringstream ms;
                    ms << "AI_ReadChannel Error Ch#" << i << " error : error_code: " << int(err);
                    message = ms.str();
                }

                chan_data[i] = chan_data_raw[i];
                chan_voltage[i] = (((F32) chan_data[i]) / AG_MAX_VALUE) * max_voltage;
                chan_data[i] = int(chan_voltage[i] * 1000);

                double total = 0.0;
                I16 cdata = chan_data_raw[i];
                switch (i) {
                    case 0:
                        UPDATE_AXIS(x, calibration0);
                        break;
                    case 1:
                        UPDATE_AXIS(y, calibration1);
                        break;
                    case 2:
                        UPDATE_AXIS(z, calibration2);
                        break;
                    case 3:
                        UPDATE_AXIS(ax, calibration3);
                        break;
                    case 4:
                        UPDATE_AXIS(ay, calibration4);
                        break;
                    case 5:
                        UPDATE_AXIS(az, calibration5);
                        break;
                }
            }

            if (filter_imu){
                if (count_filtered_imu >= SAMPLE_FILTERED_EVERY) {
                    count_filtered_imu = 1;
                    imu_raw_pub.publish(imu);
                    ROS_INFO("%s", "sending IMU filtered");
                } else {
                    count_filtered_imu++;
                }
            } else {
                imu_raw_pub.publish(imu);
                ROS_INFO("%s", "sending IMU");
            }

            // Create the IMU message and publish it
            adc::imu_Adc msg;
            msg.message = message;
            msg.x = chan_data[0];
            msg.y = chan_data[1];
            msg.z = chan_data[2];
            msg.ax = chan_data[3];
            msg.ay = chan_data[4];
            msg.az = chan_data[5];
            imu_adc_pub.publish(msg);
        }

        if ((enable_sosp) && (count_range < CICLE_RANGE)) {
            for (int i = 0; i < Chan_sosp; i++) {
                if ((err = AI_ReadChannel(card, i + 10, range_sosp, (U16*) &chan_data_raw[i])) != NoError) {
                    printf(" AI_ReadChannel Ch#%d error : error_code: %d \n", i + 10, err );
                    std::stringstream ms;
                    ms << "AI_ReadChannel Error Ch#" << i + 10 << " error : error_code: " << int(err);
                    message = ms.str();
                }

                double total = 0;
                double vRef = max_voltage;

/* FIXME: this is never entered!   it's normal, I use this piece of code in some test
                 if ((Startup)&&false) {
                 switch(i)
                 {
                 case 0:
                 calib_sosp0.push_back(float(chan_data_raw[i-10]));
                 BOOST_FOREACH( float reading, calib_sosp0 )
                 {
                 total += reading;
                 }
                 calibrationOffset[i] = total / calib_sosp0.size() - 0.800/max_voltage*maxValue;
                 ROS_INFO("Calibration channel %d",i);
                 break;
                 case 1:
                 calib_sosp1.push_back(float(chan_data_raw[i-10]));
                 BOOST_FOREACH( float reading, calib_sosp1 )
                 {
                 total += reading;
                 }
                 calibrationOffset[i] = total / calib_sosp1.size() ;
                 ROS_INFO("Calibration channel %d",i);
                 break;
                 case 2:
                 calib_sosp2.push_back(float(chan_data_raw[i-10]));
                 BOOST_FOREACH( float reading, calib_sosp2 )
                 {
                 total += reading;
                 }
                 calibrationOffset[i] = total / calib_sosp2.size() - 0.800/max_voltage*maxValue;
                 ROS_INFO("Calibration channel %d",i);
                 break;
                 case 3:
                 calib_sosp3.push_back(float(chan_data_raw[i-10]));
                 BOOST_FOREACH( float reading, calib_sosp3 )
                 {
                 total += reading;
                 }
                 calibrationOffset[i] = total / calib_sosp3.size();
                 ROS_INFO("Calibration channel %d",i);
                 break;
                 case 4:
                 calib_sosp4.push_back(float(chan_data_raw[i-10]));
                 BOOST_FOREACH( float reading, calib_sosp4 )
                 {
                 total += reading;
                 }
                 calibrationOffset[i] = total / calib_sosp4.size() - 0.800/max_voltage*maxValue;
                 ROS_INFO("Calibration channel %d",i);
                 break;
                 case 5:
                 calib_sosp5.push_back(float(chan_data_raw[i-10]));
                 BOOST_FOREACH( float reading, calib_sosp5 )
                 {
                 total += reading;
                 }
                 calibrationOffset[i] = total / calib_sosp5.size();
                 ROS_INFO("Calibration channel %d",i);
                 break;
                 case 6:
                 calib_sosp6.push_back(float(chan_data_raw[i-10]));
                 BOOST_FOREACH( float reading, calib_sosp6 )
                 {
                 total += reading;
                 }
                 calibrationOffset[i] = total / calib_sosp6.size() - 0.800/max_voltage*maxValue;
                 ROS_INFO("Calibration channel %d",i);
                 break;
                 case 7:
                 calib_sosp7.push_back(float(chan_data_raw[i-10]));
                 BOOST_FOREACH( float reading, calib_sosp7 )
                 {
                 total += reading;
                 }
                 calibrationOffset[i] = total / calib_sosp7.size();
                 ROS_INFO("Calibration channel %d",i);
                 break;
                 }
                 
                 }
                 else
                 {
                 
                 }
                 */
                 
                // These are 10 selected values from the raw output when the rover is in calibration mode
                calibrationOffset[10] = 21362;
                calibrationOffset[11] = 23414;
                calibrationOffset[12] = 20777;
                calibrationOffset[13] = 21824;
                calibrationOffset[14] = 21557;
                calibrationOffset[15] = 22494;
                calibrationOffset[16] = 20879;
                calibrationOffset[17] = 22802;

                double zeroRateV =  calibrationOffset[i+10] * vRef / AG_MAX_VALUE;
                double sensitivity = IMU2_G / G_CONSTANT; //800mv/g  [V/ m/s^2]
                chan_voltage[i] = (chan_data_raw[i] * vRef / AG_MAX_VALUE - zeroRateV) / sensitivity;
                ROS_INFO("raw %i: %i", i + 10, chan_data_raw[i]);
                ROS_INFO("tensione %i (V): %f", i + 10, chan_data_raw[i] * vRef / AG_MAX_VALUE);
                ROS_INFO("accelerazione (x,z) %i (m/s^2): %f", i + 10, chan_voltage[i]);
                ROS_INFO("accelerazione y (m/s^2): %f", rover_y);
            }

            for (int i = 0; i < Chan_sosp; i += 2) {
                double sosp_z = -chan_voltage[i];
                double sosp_x = chan_voltage[i + 1];
                // roll/pitch orientation from accelleration vector
                double sign = copysignf(1.0, sosp_z);
                double roll = atan2(rover_y, sign * sqrt(sosp_x * sosp_x + sosp_z * sosp_z));
                double pitch = -atan2(sosp_x, sqrt(rover_y * rover_y + sosp_z * sosp_z));

                // offset remove
                sign = copysignf(1.0, rover_z);
                roll += atan2(rover_y, sign * sqrt(rover_x * rover_x + rover_z * rover_z));
                pitch += -atan2(rover_x, sqrt(rover_y * rover_y + rover_z * rover_z)) ;
                sosp_data[i / 2] = pitch;

                double total = 0.0;

                switch(i / 2) {
                    case 0:
                        PITCH_UPDATE(pitch1);
                        break;
                    case 1:
                        PITCH_UPDATE(pitch2);
                        break;
                    case 2:
                        PITCH_UPDATE(pitch3);
                        break;
                    case 3:
                        PITCH_UPDATE(pitch4);
                        break;
                }
                ROS_INFO("inclinazione %d (rad): %f", (i / 2 + 1), pitch);
            }

            if (count % SAMPLE_FILTERED_EVERY == 0) {
                ROS_INFO("%s", "sending suspension");
                // Create suspension message and publish it
                adc::sosp_Adc msg;
                msg.message = message;
                msg.x1 = chan_data_raw[0];
                msg.z1 = chan_data_raw[1];
                msg.x2 = chan_data_raw[2];
                msg.z2 = chan_data_raw[3];
                msg.x3 = chan_data_raw[4];
                msg.z3 = chan_data_raw[5];
                msg.x4 = chan_data_raw[6];
                msg.z4 = chan_data_raw[7];
                msg.sosp1 = -sosp_data[0];
                msg.sosp2 = sosp_data[1];
                msg.sosp3 = sosp_data[2];
                msg.sosp4 = -sosp_data[3];
                sosp_adc_pub.publish(msg);
            }
        }

        if (enable_range) {
            // FIXME: what is this supposed to do?    only every CICLE_RANGE of the main loop enable the range finder and read it five times
            if (count_range == CICLE_RANGE) {
                DO_WriteLine(dio, 0, 0, 0);
                usleep(50000);
            }

            if (count_range >= CICLE_RANGE) {
                if (count % SAMPLE_FILTERED_EVERY_RANGE == 0) {
                    for (int i = 0; i < Chan_range; i++) {
                        if ((err = AI_ReadChannel(card, i + 6, range_range, (U16*) &chan_data_raw[i])) != NoError) {
                            printf("AI_ReadChannel Ch#%d error : error_code: %d \n", i + 6, err);
                            std::stringstream ms;
                            ms << "AI_ReadChannel Error Ch#" << i + 6 << " error : error_code: " << int(err);
                            message = ms.str();
                        }

                        chan_data[i] = chan_data_raw[i];
                        chan_voltage[i] = (((F32) chan_data[i]) / AG_MAX_VALUE) * max_voltage_range;

                        F32 cdata;
                        ROS_INFO("%s", "reading Range");
                        // range message
                        range.header.stamp = ros::Time::now();
                        range.radiation_type = 1;
                        range.field_of_view = 0.1;

                        F32 range_temp;
                        double total = 0;

                        // FIXME: please explain the constants; all of them
                        switch (i) {
                            case 0:
                                UPDATE_RANGE(range1);

                                range.header.frame_id = "Range_Post";
                                range.min_range = 0.15;
                                range.max_range = 1.50;
                                range_temp = (-32.8990*pow(cdata,3)+188.6361*pow(cdata,2)-363.4607*cdata+269.1088)/100;
                                LIMIT_RANGE();
                                rangep_pub.publish(range);
                                break;
                            case 1:
                                UPDATE_RANGE(range2);

                                range.header.frame_id = "Range_Post_Down";
                                range.min_range = 0.03;
                                range.max_range = 0.40;
                                range_temp = (-5.88513*pow(cdata,3)+36.3739*pow(cdata,2)-74.5612*cdata+56.8315)/100;
                                LIMIT_RANGE();
                                rangepd_pub.publish(range);
                                break;
                            case 2:
                                UPDATE_RANGE(range3);

                                range.header.frame_id = "Range_Front";
                                range.min_range = 0.15;
                                range.max_range = 1.50;
                                range_temp = (-32.8990*pow(cdata,3)+188.6361*pow(cdata,2)-363.4607*cdata+269.1088)/100;
                                LIMIT_RANGE();
                                rangef_pub.publish(range);
                                break;
                            case 3:
                                UPDATE_RANGE(range4);

                                range.header.frame_id = "Range_Front_Down";
                                range.min_range = 0.03;
                                range.max_range = 0.40;
                                range_temp = (-5.88513*pow(cdata,3)+36.3739*pow(cdata,2)-74.5612*cdata+56.8315)/100;
                                LIMIT_RANGE();
                                rangefd_pub.publish(range);
                                break;
                        }
                    }
                }
                count_range++;
                ROS_INFO("%i", count_range);
            }

            if (count_range >= CICLE_RANGE + (SAMPLE_FILTERED_EVERY_RANGE * SAMPLE_SIZE_5) ) {
                count_range = 0;
                DO_WriteLine(dio, 0, 0, 1);
                usleep(10000);
            }

            if (count_range < CICLE_RANGE)
                count_range ++;
        }
        
        if ((enable_imu_2) && (count_range < CICLE_RANGE)) {
            for (int i = 0; i < Chan_imu_2; i++) {
                if ((err = AI_ReadChannel(card, i + 18, range_sosp, (U16*) &chan_data_raw[i])) != NoError) {
                    printf(" AI_ReadChannel Ch#%d error : error_code: %d \n", i + 18, err);
                    std::stringstream ms;
                    ms << "AI_ReadChannel Error Ch#" << i + 18 << " error : error_code: " << int(err);
                    message = ms.str();
                }

                chan_data[i] = chan_data_raw[i];
                chan_voltage[i] = (((F32) chan_data[i]) / AG_MAX_VALUE) * max_voltage ;
                chan_data[i] = int(chan_voltage[i] * 1000);

                double total = 0.0;
                I16 cdata = chan_data_raw[i];
                switch (i) {
                    case 0:
                        UPDATE_AXIS(ax2, calibration6, i + 6, imu_2);
                        break;
                    case 1:
                        UPDATE_AXIS(ay2, calibration7, i + 6, imu_2);
                        break;
                    case 2:
                        UPDATE_AXIS(az2, calibration8, i + 6, imu_2);
                        break;
                }
            }

            if (filter_imu) {
                if (count_filtered_imu >= SAMPLE_FILTERED_EVERY) {
                    count_filtered_imu = 1;
                    imu_2_raw_pub.publish(imu_2);
                    ROS_INFO("%s", "sending IMU_2 filtered");
                } else {
                    count_filtered_imu++;
                }
            } else {
                imu_2_raw_pub.publish(imu_2);
                ROS_INFO("%s", "sending IMU_2");
            }

            // imu2 message
            adc::imu_Adc msg;
            msg.message = message;
            msg.x = 0;
            msg.y = 0;
            msg.z = 0;
            msg.ax = chan_data[0];
            msg.ay = chan_data[1];
            msg.az = chan_data[2];
            imu_2_adc_pub.publish(msg);
        }

        if (enable_diag) {
            for (int i = 0; i < Chan_diag; i++) {
                if ((err = AI_ReadChannel(card, i + 21, range_diag, (U16*) &chan_data_raw[i])) != NoError) {
                    printf(" AI_ReadChannel Ch#%d error : error_code: %d \n", i + 21, err);
                    std::stringstream ms;
                    ms << "AI_ReadChannel Error Ch#" << i + 21 << " error : error_code: " << int(err);
                    message = ms.str();
                }

                double vRef = max_voltage_diag;
                chan_voltage[i] = (chan_data_raw[i] * vRef / AG_MAX_VALUE);
                ROS_INFO("raw %i: %i", i + 21, chan_data_raw[i]);
                ROS_INFO("tensione gnd 5 3.3 %i (V): %f", i, chan_data_raw[i] * vRef / AG_MAX_VALUE);
            }

            ROS_INFO("%s", "sending diag");
            // diag message
            adc::diag_Adc msg;
            msg.message = message;
            msg.gnd = chan_voltage[0];
            msg.v5v = chan_voltage[1];
            msg.v3v3 = chan_voltage[2];
            diag_adc_pub.publish(msg);
        }

        count++;
        
        // FIXME: what is 1000?   it is a 16s timer for the initialization
        if (count > INITIALIZATION_TIMER) {
            Startup=false;
            //leave the following code commented
            //for(int i=10 ; i<10+Chan_sosp; i++ )
            //   ROS_INFO("%d  -  %f",i,cal_offset[i]);
            //return(0);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    if (card >= 0)
        Release_Card(card);
    if (dio >= 0)
        Release_Card(dio);
    ROS_INFO("Shutdown");

    return 0;
}
