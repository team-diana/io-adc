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

#define Chan_IMU 6
#define Chan_sosp 8
#define Chan_range 4
#define Chan_imu_2 3
#define Chan_diag 3

// for IMU
bool isMoving = false, Startup = true;
double cal_offset[17]={140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140};
//double orientation = 0;
double rover_x,rover_y,rover_z;

void updateImuState(sensor_msgs::Imu& imuMsg, I16 chan_data_raw, int i, boost::circular_buffer<float>& calibration)
{
    double current_time = ros::Time::now().toSec();
    double last_time = imuMsg.header.stamp.toSec();
    double linear_acceleration, angular_velocity, g;

   if (((!isMoving)&&(i<3))||(Startup)) {
       calibration.push_back(float(chan_data_raw));
       double total = 0;
       BOOST_FOREACH( float reading, calibration )
       {
           total += reading;
       }
       cal_offset[i] = total / calibration.size(); 
       ROS_INFO("Calibration channel %d",i);
   }

//    double dt = current_time - last_time;

    if ( i >= 3) //accelerometri
    {
     double scale_correction = 1.0;
     double maxValue = 32768;
     double vRef = 5;
     double zeroRateV = cal_offset[i] /* 10888*/ * vRef / maxValue;
     double sensitivity;
       //IMU1
	   if (i == 5){
             g = 0.300; //TODO da calibrare
		 	       zeroRateV -= g;
       } //su asse z aggiungo la g
       if (i <=5){
             sensitivity = 0.300/9.7803084; //300mv/g  [V/ m/s^2]
       }
       //IMU2
       if (i == 8){
             g = 0.800; //TODO da calibrare
		 	       zeroRateV -= g;
       } //su asse z aggiungo la g
       if ((i <=8)&&(i >5)){
             sensitivity = 0.800/9.7803084; //300mv/g  [V/ m/s^2]
       }
     linear_acceleration = (chan_data_raw * vRef / maxValue - zeroRateV) / sensitivity;
    }
    
    if ( i < 3) //giroscopi
    {
       double scale_correction = 1.0;
       double maxValue = 32768;
       double vRef = 5;
       double zeroRateV = cal_offset[i] /* 8340*/ * vRef / maxValue;
       double sensitivity = 0.00333/(2*M_PI/360.0); //3.33mv/°/s  [V/rad/s]
       angular_velocity = (chan_data_raw * vRef / maxValue - zeroRateV) / sensitivity;
    }

//    drate = (chan_data_raw * vRef / maxValue - zeroRateV) / sensitivity;
//
//    orientation += drate * dt;
    //ROS_INFO("orientation (deg): %f", (orientation));
    //ROS_INFO("orientation (rad): %f", (orientation * (M_PI/180.0)));

    imuMsg.header.stamp = ros::Time::now();
//    imuMsg.orientation = tf::createQuaternionMsgFromYaw(orientation * (M_PI/180.0));
    switch (i)
     {
    	case(0)://x
    		    imuMsg.angular_velocity.y = angular_velocity;
    	break;
    	case(1)://y
    		    imuMsg.angular_velocity.x = -angular_velocity;
    	break;
    	case(2)://z
    		    imuMsg.angular_velocity.z = angular_velocity;
    	break;
        case(3)://ax
    		    imuMsg.linear_acceleration.y = linear_acceleration;
                rover_x = linear_acceleration;
    	break;
    	case(4)://ay
    		    imuMsg.linear_acceleration.x = -linear_acceleration;
                rover_y = linear_acceleration;
    	break;
    	case(5)://az
    		    imuMsg.linear_acceleration.z = linear_acceleration;
                rover_z = linear_acceleration;
    	break;
      case(6):
    		    imuMsg.linear_acceleration.x = linear_acceleration;
    	break;
    	case(7):
    		    imuMsg.linear_acceleration.y = linear_acceleration;
                    //rover_y = linear_acceleration;
    	break;
    	case(8):
    		    imuMsg.linear_acceleration.z = linear_acceleration;
    	break;
     }
}


bool movingStatus(adc::movingService::Request &req, adc::movingService::Response &res)
{
     isMoving = req.status;
     ros::param::set("/moving", isMoving);
     return true;
}



/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  std::cout << "start" << std::endl;
  ros::init(argc, argv, "adc");
  ros::NodeHandle n;
  
  // IMU message
  sensor_msgs::Imu imu,imu_2;
  sensor_msgs::Range range;
  boost::circular_buffer<float> calibration0(140),calibration1(140),calibration2(140),calibration3(140),calibration4(140),calibration5(140),calibration6(140),calibration7(140),calibration8(140),range1(6),range2(6),range3(6),range4(6),pitch1(400),pitch2(400),pitch3(400),pitch4(400),x(50),y(50),z(50),ax(50),ay(50),az(50),ax2(50),ay2(50),az2(50),calib_sosp0(140),calib_sosp1(140),calib_sosp2(140),calib_sosp3(140),calib_sosp4(140),calib_sosp5(140),calib_sosp6(140),calib_sosp7(140);

  // Declare variables that can be modified by launch file.
  std::string message;
  int rate;
  
  std::string topic_imu, topic_sosp, topic_rangef, topic_rangefd, topic_rangep, topic_rangepd, topic_imu_2, topic_diag;
  int range_imu, range_sosp, range_range, range_imu_2, range_diag;
  bool enable_imu, enable_sosp, enable_range, filter_imu, enable_imu_2, enable_diag;
  
  int i;
  I16 card=-1, err, dio=-1;
  char temp_char;
  I16 chan_data[8];                     // TODO configurare meglio il massimo del buffer
  I16 chan_data_raw[8];
  F32 sosp_data[4];
  F32 chan_voltage[8], max_voltage, max_voltage_range, max_voltage_diag;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("message", message, std::string("NO error"));
  private_node_handle_.param("rate", rate, int(400));
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
  
  
  
  
  if ((card=Register_Card (PCI_9116, 0)) <0 ) 
  {
    std::cout <<  "ADC Register_Card error= " <<  card << std::endl;
		message = "ADC Register_Card error";
    exit(1);
  }
  if (err = AI_9116_Config(card, P9116_AI_SingEnded|P9116_AI_UserCMMD, P9116_AI_SoftPolling ,0, 0, 0) != NoError)
  {
    printf(" Config error : error_code: %d \n", err );
    std::stringstream ms;
		ms << "Config error Error: error_code: " << int(err);
    message =ms.str();
  }
  
  if ((dio=Register_Card (PCI_7432, 0)) <0 ) 
  {
    std::cout <<  "DIO Register_Card error= " <<  dio << std::endl;
		message = "DIO Register_Card error";
    exit(1);
  }
  
  
  
  
  if (enable_imu)
  {
      switch (range_imu)
      {
       	case(2):
       		max_voltage=5;
       	break;
       	case(3):
       		max_voltage=2.5;
       	break;
       	case(4):
       		max_voltage=1.25;
       	break;
      }
      //TODO dare un colpo di HP reset ad ogni accensione tramite DIO
      //DO_WriteLine(dio, 0,0, 1);
      //usleep(25000000);
      //DO_WriteLine(dio, 0,0, 0);
  }
  
  if (enable_sosp)
  {
      switch (range_imu)
      {
       	case(2):
       		max_voltage=5;
       	break;
       	case(3):
       		max_voltage=2.5;
       	break;
       	case(4):
       		max_voltage=1.25;
       	break;
      }
  }  
  
  if (enable_range)
  {
      switch (range_range)
      {
       	case(2):
       		max_voltage_range=5;
       	break;
       	case(3):
       		max_voltage_range=2.5;
       	break;
       	case(4):
       		max_voltage_range=1.25;
       	break;
      }
  } 
  
  if (enable_imu_2)
  {
      switch (range_imu_2)
      {
       	case(2):
       		max_voltage=5;
       	break;
       	case(3):
       		max_voltage=2.5;
       	break;
       	case(4):
       		max_voltage=1.25;
       	break;
      }  
  }
  
  if (enable_diag)
  {
      switch (range_diag)
      {
       	case(2):
       		max_voltage_diag=5;
       	break;
       	case(3):
       		max_voltage_diag=2.5;
       	break;
       	case(4):
       		max_voltage_diag=1.25;
       	break;
      }  
  }
  
  
  
  // TODO config
  // fill in imu data (start at 0 orientation)
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
  while (n.ok())
  {
    bool filter_imu_temp = filter_imu;
    if (!n.getParam ("/moving", isMoving))
        {
        isMoving = false;
        n.setParam("/moving", bool(false));
        }
    if (!n.getParam ("/adc/filtered_imu", filter_imu))
        {
        filter_imu = false;
        n.setParam("/adc/filtered_imu", bool(false));
        if ((filter_imu_temp != filter_imu)&&(filter_imu))
           count = 0;
        }
  
    if ((enable_imu) && (count_range<370))
      {
        	for(int i=0 ; i<Chan_IMU; i++ )
        	{
           if( (err = AI_ReadChannel(card, i, range_imu, (U16*)&chan_data_raw[i]) ) != NoError )
        		{
              printf(" AI_ReadChannel Ch#%d error : error_code: %d \n", i, err );
              std::stringstream ms;
        			ms << "AI_ReadChannel Error Ch#" << i << " error : error_code: " << int(err);
              message =ms.str();
        		}
           chan_data[i] = chan_data_raw[i];
           chan_voltage[i] = (((F32) chan_data[i]) / 32768)* max_voltage ;
           chan_data[i] = int (chan_voltage[i] * 1000);
           
           double total = 0.0;
           I16 cdata=chan_data_raw[i];
      	   switch (i)
      		  {
      		  case 0:
              if (filter_imu)
              {
//                 BOOST_FOREACH( float reading, x )
//                 {
//                     total += reading;
//                 }
//                 float median = total / x.size();
//                 if ((abs(cdata - median)/median < 0.30) || (count == 0))
//                     x.push_back(float(cdata));
//                 else
//                     {
//                     ROS_INFO("--------------------------------------------------------------------x");
//                     cdata = median;
		         x.push_back(float(cdata));
//                     }
                 BOOST_FOREACH( float reading, x )
                 {
                     total += reading;
                 }
                 cdata = total / x.size(); 
               }
      			  updateImuState(imu, cdata, i, calibration0);
      		  break;
      		  case 1:
              if (filter_imu)
              {
//                 BOOST_FOREACH( float reading, y )
//                 {
//                     total += reading;
//                 }
//                 float median = total / y.size();
//                 if ((abs(cdata - median)/median < 0.30) || (count == 0))
//                     y.push_back(float(cdata));
//                 else
//                     {
//                     ROS_INFO("--------------------------------------------------------------------y");
//                     cdata = median;
                 y.push_back(float(cdata));
//                     }
                 BOOST_FOREACH( float reading, y )
                 {
                     total += reading;
                 }
                 cdata = total / y.size(); 
               }
      			updateImuState(imu, cdata, i, calibration1);
      		  break;
      		  case 2:
              if (filter_imu)
              {
//                 BOOST_FOREACH( float reading, z )
//                 {
//                     total += reading;
//                 }
//                 float median = total / z.size();
//                 if ((abs(cdata - median)/median < 0.30) || (count == 0))
//                     z.push_back(float(cdata));
//                 else
//                     {
//                     ROS_INFO("--------------------------------------------------------------------z");
//                     cdata = median;
		         z.push_back(float(cdata));
//                     }
                 BOOST_FOREACH( float reading, z )
                 {
                     total += reading;
                 }
                 cdata = total / z.size(); 
               }
      			updateImuState(imu, cdata, i, calibration2);
      		  break;
      		  case 3:
              if (filter_imu)
              {
//                 BOOST_FOREACH( float reading, ax )
//                 {
//                     total += reading;
//                 }
//                 float median = total / ax.size();
//                 if ((abs(cdata - median)/median < 0.04) || (count == 0))
//                     ax.push_back(float(cdata));
//                 else
//                     {
//                     ROS_INFO("--------------------------------------------------------------------ax");
//                     cdata = median;
                 ax.push_back(float(cdata));
//                     }
                 BOOST_FOREACH( float reading, ax )
                 {
                     total += reading;
                 }
                 cdata = total / ax.size(); 
               }
      			updateImuState(imu, cdata, i, calibration3);
      		  break;
      		  case 4:
              if (filter_imu)
              {
//                 BOOST_FOREACH( float reading, ay )
//                 {
//                     total += reading;
//                 }
//                 float median = total / ay.size();
//                 if ((abs(cdata - median)/median < 0.04) || (count == 0))
//                     ay.push_back(float(cdata));
//                 else
//                     {
//                     ROS_INFO("--------------------------------------------------------------------ay");
//                     cdata = median;
                 ay.push_back(float(cdata));
//                     }
                 BOOST_FOREACH( float reading, ay )
                 {
                     total += reading;
                 }
                 cdata = total / ay.size(); 
               }
      			updateImuState(imu, cdata, i, calibration4);
      		  break;
      		  case 5:
              if (filter_imu)
              {
//                 BOOST_FOREACH( float reading, az )
//                 {
//                     total += reading;
//                 }
//                 float median = total / az.size();
//                 if ((abs(cdata - median)/median < 0.04) || (count == 0))
//                     az.push_back(float(cdata));
//                 else
//                     {
//                     ROS_INFO("--------------------------------------------------------------------az");
//                     cdata = median;
                 az.push_back(float(cdata));
//                     }
                 BOOST_FOREACH( float reading, az )
                 {
                     total += reading;
                 }
                 cdata = total / az.size(); 
               }
      			updateImuState(imu, cdata, i, calibration5);
      		  break;
      		  }
         }
             if (filter_imu)
                  {
                      if (count_filtered_imu>=10)
                      {
                         count_filtered_imu = 1;
                         imu_raw_pub.publish(imu); 
                         ROS_INFO("%s", "sending IMU filtered");
                      }
                      else
                      {
                           count_filtered_imu ++;
                      }
                   }
             else
                  {
                  imu_raw_pub.publish(imu);
                  ROS_INFO("%s", "sending IMU"); 
                  }
             // imu message
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
      
      
      
      
      
    if ((enable_sosp) && (count_range<370))
      {

         for(int i=10 ; i<10+Chan_sosp; i++ )
         {
              if( (err = AI_ReadChannel(card, i, range_sosp, (U16*)&chan_data_raw[i-10]) ) != NoError )
           		{
                 printf(" AI_ReadChannel Ch#%d error : error_code: %d \n", i, err );
                 std::stringstream ms;
           			ms << "AI_ReadChannel Error Ch#" << i << " error : error_code: " << int(err);
                 message =ms.str();
           		}
       //       chan_data[i-6] = chan_data_raw[i-6];
       //       chan_voltage[i-6] = (((F32) chan_data[i-6]) / 32768)* max_voltage ;
       //       chan_data[i-6] = int (chan_voltage[i-6] * 1000);
              
              //I16 cdata=chan_data_raw[i];
              //updateSospState(imu, cdata, i, calibration);
              //sosp_raw_pub.publish(sosp);
              
                double total = 0;
                double scale_correction = 1.0;
                double maxValue = 32768;
                double vRef = max_voltage;
              
                if ((Startup)||false) {
                   switch(i)
                   {
                        case 10:
                           calib_sosp0.push_back(float(chan_data_raw[i-10]));
                           BOOST_FOREACH( float reading, calib_sosp0 )
                           {
                               total += reading;
                           }
                           cal_offset[i] = total / calib_sosp0.size() - 0.800/max_voltage*maxValue; 
                           ROS_INFO("Calibration channel %d",i);
                        break;
                        case 11:
                           calib_sosp1.push_back(float(chan_data_raw[i-10]));
                           BOOST_FOREACH( float reading, calib_sosp1 )
                           {
                               total += reading;
                           }
                           cal_offset[i] = total / calib_sosp1.size() ; 
                           ROS_INFO("Calibration channel %d",i);
                        break;
                        case 12:
                           calib_sosp2.push_back(float(chan_data_raw[i-10]));
                           BOOST_FOREACH( float reading, calib_sosp2 )
                           {
                               total += reading;
                           }
                           cal_offset[i] = total / calib_sosp2.size() - 0.800/max_voltage*maxValue; 
                           ROS_INFO("Calibration channel %d",i);
                        break;
                        case 13:
                           calib_sosp3.push_back(float(chan_data_raw[i-10]));
                           BOOST_FOREACH( float reading, calib_sosp3 )
                           {
                               total += reading;
                           }
                           cal_offset[i] = total / calib_sosp3.size(); 
                           ROS_INFO("Calibration channel %d",i);
                        break;
                        case 14:
                           calib_sosp4.push_back(float(chan_data_raw[i-10]));
                           BOOST_FOREACH( float reading, calib_sosp4 )
                           {
                               total += reading;
                           }
                           cal_offset[i] = total / calib_sosp4.size() - 0.800/max_voltage*maxValue; 
                           ROS_INFO("Calibration channel %d",i);
                        break;
                        case 15:
                           calib_sosp5.push_back(float(chan_data_raw[i-10]));
                           BOOST_FOREACH( float reading, calib_sosp5 )
                           {
                               total += reading;
                           }
                           cal_offset[i] = total / calib_sosp5.size(); 
                           ROS_INFO("Calibration channel %d",i);
                        break;
                        case 16:
                           calib_sosp6.push_back(float(chan_data_raw[i-10]));
                           BOOST_FOREACH( float reading, calib_sosp6 )
                           {
                               total += reading;
                           }
                           cal_offset[i] = total / calib_sosp6.size() - 0.800/max_voltage*maxValue; 
                           ROS_INFO("Calibration channel %d",i);
                        break;
                        case 17:
                           calib_sosp7.push_back(float(chan_data_raw[i-10]));
                           BOOST_FOREACH( float reading, calib_sosp7 )
                           {
                               total += reading;
                           }
                           cal_offset[i] = total / calib_sosp7.size(); 
                           ROS_INFO("Calibration channel %d",i);
                        break;
                   }
                   
                }
                else
                    {
                    cal_offset[10]=21000;
                    cal_offset[11]=21000;
                    cal_offset[12]=21000;
                    cal_offset[13]=21000;
                    cal_offset[14]=21000;
                    cal_offset[15]=21000;
                    cal_offset[16]=21000;
                    cal_offset[17]=21000;
                    }
              
              double zeroRateV =  cal_offset[i] * vRef / maxValue;
              double sensitivity = 0.800/9.7803084; //800mv/g  [V/ m/s^2]
              chan_voltage[i-10] = (chan_data_raw[i-10] * vRef / maxValue - zeroRateV) / sensitivity;
       			  ROS_INFO("raw %i: %i", i ,chan_data_raw[i-10]);
       			  ROS_INFO("tensione %i (V): %f", i ,chan_data_raw[i-10] * vRef / maxValue);
       			  ROS_INFO("accelerazione (x,z) %i (m/s^2): %f", i ,chan_voltage[i-10]);
                  ROS_INFO("accelerazione y (m/s^2): %f", rover_y);

         }
         
         for(int i=0 ; i<Chan_sosp; i=i+2 )
         {
           double sosp_z = -chan_voltage[i];
           double sosp_x = chan_voltage[i+1];
           // roll/pitch orientation from acc. vector    
           double sign = copysignf(1.0, sosp_z);
           double roll = atan2(rover_y, sign * sqrt(sosp_x*sosp_x + sosp_z*sosp_z));
           double pitch = -atan2(sosp_x, sqrt(rover_y*rover_y + sosp_z*sosp_z));
           // offset remove
           sign = copysignf(1.0, rover_z);
           roll += atan2(rover_y, sign * sqrt(rover_x*rover_x + rover_z*rover_z));
           pitch += -atan2(rover_x, sqrt(rover_y*rover_y + rover_z*rover_z)) ;
           sosp_data[i/2] = pitch;
		   
       double total = 0.0;
       
       switch(i/2)
		   {
		       case 0:
			       pitch1.push_back(float(pitch));
			       BOOST_FOREACH( float reading, pitch1 )
				   {
					   total += reading;
				   }
				   sosp_data[i/2] = total / pitch1.size();
			   break;
			   case 1:
			       pitch2.push_back(float(pitch));
			       BOOST_FOREACH( float reading, pitch2 )
				   {
					   total += reading;
				   }
				   sosp_data[i/2] = total / pitch2.size();
			   break;
			   case 2:
			       pitch3.push_back(float(pitch));
			       BOOST_FOREACH( float reading, pitch3 )
				   {
					   total += reading;
				   }
				   sosp_data[i/2] = total / pitch3.size();
			   break;
			   case 3:
			       pitch4.push_back(float(pitch));
			       BOOST_FOREACH( float reading, pitch4 )
				   {
					   total += reading;
				   }
				   sosp_data[i/2] = total / pitch4.size();
			   break;
		   }
		   ROS_INFO("inclinazione %d (rad): %f", (i/2+1) ,pitch);
         }
     	 
         if (count%10 == 0)
         {
             ROS_INFO("%s", "sending suspension"); 
             // sosp message
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
             msg.sosp1 = - sosp_data[0];
             msg.sosp2 = sosp_data[1];
             msg.sosp3 = sosp_data[2];
             msg.sosp4 = - sosp_data[3];
             sosp_adc_pub.publish(msg);
         }
      }
      
      
      
      
    if (enable_range)
      {
          if (count_range==370)
          {
             DO_WriteLine(dio, 0,0, 0);
             usleep(50000);
          }
          if (count_range>=370)
          {
             if (count%10 == 0)
             {
                for(int i=6 ; i<Chan_range+6; i++ )
                {
                   if( (err = AI_ReadChannel(card, i, range_range, (U16*)&chan_data_raw[i-6]) ) != NoError )
                        {
                         printf(" AI_ReadChannel Ch#%d error : error_code: %d \n", i, err );
                         std::stringstream ms;
                         ms << "AI_ReadChannel Error Ch#" << i << " error : error_code: " << int(err);
                         message =ms.str();
                        }
                   chan_data[i-6] = chan_data_raw[i-6];
                   chan_voltage[i-6] = (((F32) chan_data[i-6]) / 32768)* max_voltage_range ;
                   //chan_data[i-6] = int (chan_voltage[i-6] * 1000);
                   
                   //ROS_INFO("lettura %d (V): %f", i, (chan_voltage[i-6]));
                   
                   F32 cdata;//=chan_voltage[i-6];
                   ROS_INFO("%s", "reading Range"); 
                   // adc message
                   range.header.stamp = ros::Time::now();
                   range.radiation_type = 1;
                   range.field_of_view = 0.1;
                   
                   F32 range_temp;
                   double total = 0;
                   
                   
                   switch (i-6)
                      {
                      case 2:
                       range3.push_back(float(chan_voltage[i-6]));
                       BOOST_FOREACH( float reading, range3 )
                       {
                           total += reading;
                       }
                       cdata = total / range3.size(); 
                       
                       range.header.frame_id = "Range_Front";
                         range.min_range = 0.15;
                       range.max_range = 1.50;
                       range_temp = (-32.8990*pow(cdata,3)+188.6361*pow(cdata,2)-363.4607*cdata+269.1088)/100;
                       if (range_temp > range.max_range)
                            range.range = range.max_range;
                       else if (range_temp < range.min_range)
                            range.range = range.min_range;
                       else
                            range.range = range_temp;
                       rangef_pub.publish(range);
                      break;
                      case 3:
                       range4.push_back(float(chan_voltage[i-6]));
                       BOOST_FOREACH( float reading, range4 )
                       {
                           total += reading;
                       }
                       cdata = total / range4.size(); 
                       
                       range.header.frame_id = "Range_Front_Down";
                             range.min_range = 0.03;
                       range.max_range = 0.40;
                       range_temp = (-5.88513*pow(cdata,3)+36.3739*pow(cdata,2)-74.5612*cdata+56.8315)/100;
                       if (range_temp > range.max_range)
                            range.range = range.max_range;
                       else if (range_temp < range.min_range)
                            range.range = range.min_range;
                       else
                            range.range = range_temp;
                       rangefd_pub.publish(range);
                      break;
                      case 0:
                       range1.push_back(float(chan_voltage[i-6]));
                       BOOST_FOREACH( float reading, range1 )
                       {
                           total += reading;
                       }
                       cdata = total / range1.size(); 
                       
                       range.header.frame_id = "Range_Post";
                           range.min_range = 0.15;
                       range.max_range = 1.50;
                       range_temp = (-32.8990*pow(cdata,3)+188.6361*pow(cdata,2)-363.4607*cdata+269.1088)/100;
                       if (range_temp > range.max_range)
                            range.range = range.max_range;
                       else if (range_temp < range.min_range)
                            range.range = range.min_range;
                       else
                            range.range = range_temp;
                       rangep_pub.publish(range);
                      break;
                      case 1:
                       range2.push_back(float(chan_voltage[i-6]));
                       BOOST_FOREACH( float reading, range2 )
                       {
                           total += reading;
                       }
                       cdata = total / range2.size(); 
                       
                       range.header.frame_id = "Range_Post_Down";
                             range.min_range = 0.03;
                       range.max_range = 0.40;
                       range_temp = (-5.88513*pow(cdata,3)+36.3739*pow(cdata,2)-74.5612*cdata+56.8315)/100;
                       if (range_temp > range.max_range)
                            range.range = range.max_range;
                       else if (range_temp < range.min_range)
                            range.range = range.min_range;
                       else
                            range.range = range_temp;
                       rangepd_pub.publish(range);
                      break;
                      }
                 }
             }
             count_range ++;
             ROS_INFO("%i",count_range);
          }
          if (count_range>=400)
          {
             count_range = 0; 
             DO_WriteLine(dio, 0,0,1); 
             usleep(10000);
          }
          if (count_range<370)
          {
             count_range ++;
          }
      }      
      
      
      
      
      
    if ((enable_imu_2) && (count_range<370))
      {
         for(int i=18 ; i<18+Chan_imu_2; i++ )
         {
              if( (err = AI_ReadChannel(card, i, range_sosp, (U16*)&chan_data_raw[i-18]) ) != NoError )
           		{
                 printf(" AI_ReadChannel Ch#%d error : error_code: %d \n", i, err );
                 std::stringstream ms;
           			ms << "AI_ReadChannel Error Ch#" << i << " error : error_code: " << int(err);
                 message =ms.str();
           		}
           chan_data[i-18] = chan_data_raw[i-18];
           chan_voltage[i-18] = (((F32) chan_data[i-18]) / 32768)* max_voltage ;
           chan_data[i-18] = int (chan_voltage[i-18] * 1000);
           
           double total = 0.0;
           I16 cdata=chan_data_raw[i-18];
      	   switch (i-18)
      		  {
      		  case 0:
              if (filter_imu)
              {
//                 BOOST_FOREACH( float reading, ax )
//                 {
//                     total += reading;
//                 }
//                 float median = total / ax.size();
//                 if ((abs(cdata - median)/median < 0.04) || (count == 0))
//                     ax.push_back(float(cdata));
//                 else
//                     {
//                     ROS_INFO("--------------------------------------------------------------------ax");
//                     cdata = median;
                 ax2.push_back(float(cdata));
//                     }
                 BOOST_FOREACH( float reading, ax2 )
                 {
                     total += reading;
                 }
                 cdata = total / ax2.size(); 
               }
      			updateImuState(imu_2, cdata, i-16+6, calibration6);
      		  break;
      		  case 1:
              if (filter_imu)
              {
//                 BOOST_FOREACH( float reading, ay )
//                 {
//                     total += reading;
//                 }
//                 float median = total / ay.size();
//                 if ((abs(cdata - median)/median < 0.04) || (count == 0))
//                     ay.push_back(float(cdata));
//                 else
//                     {
//                     ROS_INFO("--------------------------------------------------------------------ay");
//                     cdata = median;
                 ay2.push_back(float(cdata));
//                     }
                 BOOST_FOREACH( float reading, ay2 )
                 {
                     total += reading;
                 }
                 cdata = total / ay2.size(); 
               }
      			updateImuState(imu_2, cdata, i-16+6, calibration7);
      		  break;
      		  case 2:
              if (filter_imu)
              {
//                 BOOST_FOREACH( float reading, az )
//                 {
//                     total += reading;
//                 }
//                 float median = total / az.size();
//                 if ((abs(cdata - median)/median < 0.04) || (count == 0))
//                     az.push_back(float(cdata));
//                 else
//                     {
//                     ROS_INFO("--------------------------------------------------------------------az");
//                     cdata = median;
                 az2.push_back(float(cdata));
//                     }
                 BOOST_FOREACH( float reading, az2 )
                 {
                     total += reading;
                 }
                 cdata = total / az2.size(); 
               }
      			updateImuState(imu_2, cdata, i-16+6, calibration8);
      		  break;
      		  }
         }
             if (filter_imu)
                  {
                      if (count_filtered_imu>=10)
                      {
                         count_filtered_imu = 1;
                         imu_2_raw_pub.publish(imu_2); 
                         ROS_INFO("%s", "sending IMU_2 filtered");
                      }
                      else
                      {
                           count_filtered_imu ++;
                      }
                   }
             else
                  {
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
      
      
      
      
      
    if ((enable_diag))
    {
         for(int i=21 ; i<21+Chan_diag; i ++ )
         {
              if( (err = AI_ReadChannel(card, i, range_diag, (U16*)&chan_data_raw[i-21]) ) != NoError )
           		{
                 printf(" AI_ReadChannel Ch#%d error : error_code: %d \n", i, err );
                 std::stringstream ms;
           			ms << "AI_ReadChannel Error Ch#" << i << " error : error_code: " << int(err);
                 message =ms.str();
           		}

              double scale_correction = 1.0;
              double maxValue = 32768;
              double vRef = max_voltage_diag;
              chan_voltage[i-21] = (chan_data_raw[i-21] * vRef / maxValue);
       			  ROS_INFO("raw %i: %i", i ,chan_data_raw[i-21]);
       			  ROS_INFO("tensione gnd 5 3.3 %i (V): %f", i-21 ,chan_data_raw[i-21] * vRef / maxValue);
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
      

	++count;
 
    if (count>2000)
    {
        Startup=false;
        //for(int i=10 ; i<10+Chan_sosp; i++ )
        //   ROS_INFO("%d  -  %f",i,cal_offset[i])
        //return(0)
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  if(card>=0)
    Release_Card( card );
  if(dio>=0)
    Release_Card( dio );
  ROS_INFO("Shutdown");
  
  return 0;
}
 // end main()
