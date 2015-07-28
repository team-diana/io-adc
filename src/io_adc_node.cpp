#include "io_adc/io_adc_node.hpp"
#include "io_adc/dask.h"
#include "io_adc/utils.hpp"

#include "io_adc/sosp_Adc.h"
#include "io_adc/wheel_motor_temperature.h"

#include <team_diana_lib/strings/strings.h>
#include <thread>

using namespace io_adc;

enum  {
  MOTOR_TEMP_SENS_PIN_BASE = 13
};

IoAdcNode::IoAdcNode() : nodeHandle("io_adc")
{
    P9116Params = getP9116ParamsFromRosParams(nodeHandle);

    suspensionPublisher = nodeHandle.advertise<io_adc::sosp_Adc>("sospension_absolute", 100);
}

IoAdcNode::~IoAdcNode()
{
    ROS_INFO("Releasing cards");

    Release_Card(ioCard);
    Release_Card(adcCard);
}


bool IoAdcNode::init()
{
    uint16_t err;

    ROS_INFO("Init PCI_9116");
    if ((adcCard = Register_Card(PCI_9116, 0)) < 0) {
        log_register_card_error("PCI_9116", adcCard);
        return false;
    }

    ROS_INFO("Setup PCI_9116");
    if ((err = AI_9116_Config(adcCard, P9116Params.getConfigCtrlValue(), P9116_AI_SoftPolling ,0, 0, 0)) != NoError) {
        log_register_card_error("PCI_9116 configure error", err);
        return false;
    }

    ROS_INFO("Init PCI_7432");
    if ((ioCard = Register_Card(PCI_7432, 0)) < 0) {
        log_register_card_error("pCI_7432", ioCard);
        return false;
    }

    ROS_INFO("Init done");

    return true;
}

void IoAdcNode::setupReaders()
{
    setupSuspensionReaders();
    motorTempSensor.setup(nodeHandle);
}

void IoAdcNode::setupSuspensionReaders()
{
    ros::NodeHandle privateNodehandle("~");

    SuspensionPortConf suspension1Conf;

    int adcPort;
    privateNodehandle.param("suspension_1_port_x", adcPort, 10);
    suspension1Conf.xAdcPort = adcPort;
    privateNodehandle.param("suspension_1_port_z", adcPort, 11);
    suspension1Conf.zAdcPort = adcPort;

    suspenionReader1 = std::unique_ptr<SuspensionReader>( new SuspensionReader(suspension1Conf, "suspension_1_back_right"));
}

void IoAdcNode::run()
{
    ROS_INFO("Run");

    while(ros::ok()) {
        updateSuspensions();
        motorTempSensor.readNewValues(adcCard);
        motorTempSensor.publishNewValues();
        usleep(1000);
        ros::spinOnce();
    }

    ROS_INFO("Stop");
}

void IoAdcNode::updateSuspensions()
{
    SuspensionValue value1;

    if(suspenionReader1) {
        suspenionReader1->update(adcCard);
        value1 = suspenionReader1->getValue();
    }

    publishSuspension(value1);
}

void IoAdcNode::publishSuspension(SuspensionValue suspensionValue1)
{
    io_adc::sosp_Adc msg;
    msg.x1 = suspensionValue1.x;
    msg.z1 = suspensionValue1.z;

    suspensionPublisher.publish(msg);
}

void IoAdcNode::updateMotorTemperatureSensor()
{
  io_adc::wheel_motor_temperature msg;


}



int main(int argc, char** argv) {
    ros::init(argc, argv, "io_adc");

    IoAdcNode ioAdcNode;

    if(!ioAdcNode.init()) {
        ROS_ERROR("error in init, exiting");
    }

    ioAdcNode.run();
}

void MotorTempSensor::readNewValues(uint16_t adcCard)
{
  for(int i = 0; i < 4; i++) {
    uint16_t err;
    int16_t rawValue;
    uint16_t range = AD_U_5_V;
    double converted;

    if ((err = AI_ReadChannel(adcCard, MOTOR_TEMP_SENS_PIN_BASE+i,
          range, (uint16_t*)&rawValue) ) != NoError) {
      log_input_error_adc(i, "while reading raw value", err);
    } else {
      AI_VoltScale(adcCard, range, rawValue, &converted);
      motTempBuffers[i].append(rawValue);
    }
  }
}

void MotorTempSensor::publishNewValues()
{
  std::vector<float> temps;
  for(int i = 0; i < 4; i++) {
    float avg = motTempBuffers[i].average();
    float temp = motTempSensCalibrations[i].baseTemp +
      (motTempSensCalibrations[i].baseTempVolt - avg) * motTempSensCalibrations[i].tempVoltDiff;
    temps.push_back(temp);
  }
  io_adc::wheel_motor_temperature msg;
  msg.temp = temps;
  motorTemperaturePublisher.publish(msg);
}


void MotorTempSensor::setup(ros::NodeHandle nodeHandle)
{
    for(int i = 0; i<4; i++) {
      std::string base = Td::toString("temp_sensor_calibration_motor_", i, "/");

      auto baseTemp = base + "base_temp";
      auto baseTempV = base + "base_temp_v";
      auto dangerV = base + "danger_v";
      auto tempVoltDiff = base + "temp_volt_diff";

      checkParamExistenceOrExit(nodeHandle, baseTemp);
      checkParamExistenceOrExit(nodeHandle, baseTempV);
      checkParamExistenceOrExit(nodeHandle, dangerV);
      checkParamExistenceOrExit(nodeHandle, tempVoltDiff);

      nodeHandle.param(baseTemp, motTempSensCalibrations[i].baseTemp, 0.0);
      nodeHandle.param(baseTempV, motTempSensCalibrations[i].baseTempVolt, 0.0);
      nodeHandle.param(dangerV, motTempSensCalibrations[i].dangerVolt, 0.0);
      nodeHandle.param(tempVoltDiff, motTempSensCalibrations[i].tempVoltDiff, 0.0);
      motorTemperaturePublisher = nodeHandle.advertise<io_adc::wheel_motor_temperature>(
        Td::toString("motor_temperature_sensor"), 100);
    }

}
