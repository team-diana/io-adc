#include "io_adc/io_adc_node.hpp"
#include "io_adc/dask.h"
#include "io_adc/utils.hpp"

#include "io_adc/sosp_Adc.h"
#include "io_adc/wheel_motor_temperature.h"

#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/logging/logging.h>
#include <thread>

using namespace io_adc;
using namespace std;

IoAdcNode::IoAdcNode() : nodeHandle("io_adc")
{
    P9116Params = getP9116ParamsFromRosParams(nodeHandle);
    int sleepTimeMs;
    nodeHandle.param("sleep_time_ms", sleepTimeMs, 100);
    sleepTime = std::chrono::milliseconds(sleepTimeMs);
    suspensionPublisher = nodeHandle.advertise<io_adc::sosp_Adc>("sospension_absolute", 100);
    enableSuspensionPower = nodeHandle.advertiseService("enable_suspension_power",
                                                &IoAdcNode::setEnableSuspensionPowerCallback, this);
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

    Td::ros_info("Init PCI_9116");
    if ((adcCard = Register_Card(PCI_9116, 0)) < 0) {
        log_register_card_error("PCI_9116", adcCard);
        return false;
    }

    Td::ros_info(Td::toString("Setup PCI_9116: ", P9116Params.toString()));
    if ((err = AI_9116_Config(adcCard, P9116Params.getConfigCtrlValue(), P9116_AI_SoftPolling ,0, 0, 0)) != NoError) {
        log_register_card_error("PCI_9116 configure error", err);
        return false;
    }

    Td::ros_info("Init PCI_7432");
    if ((ioCard = Register_Card(PCI_7432, 0)) < 0) {
        log_register_card_error("pCI_7432", ioCard);
        return false;
    }

    Td::ros_info("Setup readers");
    if(!setupReaders()) {
      Td::ros_error("Invalid configuration");
      return false;
    }

    Td::ros_info("Init done");

    return true;
}

bool IoAdcNode::setupReaders()
{
    if(!setupSuspensionReaders()) {
      return false;
    }
    if(!setupCurrentReaders()) {
      return false;
    }
    motorTempSensor.setup(nodeHandle);
    return true;
}

bool IoAdcNode::setupSuspensionReaders()
{
    ros::NodeHandle privateNodehandle("~");

    SuspensionPortConf suspension1Conf;

    int adcPort;
    privateNodehandle.param("suspension_1_port_x", adcPort, 10);
    suspension1Conf.xAdcPort = adcPort;
    privateNodehandle.param("suspension_1_port_z", adcPort, 11);
    suspension1Conf.zAdcPort = adcPort;

    suspenionReader1 = std::unique_ptr<SuspensionReader>( new SuspensionReader(suspension1Conf, "suspension_1_back_right"));
    return true;
}

bool IoAdcNode::setupCurrentReaders()
{
    ros::NodeHandle privateNodehandle("~");

    CurrentReaderConf conf;

    vector<string> current_sensors;

    current_sensors = privateNodehandle.param("current_sensors", current_sensors);

    if(current_sensors.empty()) {
      ROS_INFO("No current sensor specified");
    } else {
      for(const string name : current_sensors) {
        ROS_INFO("Setting up current sensor '%s'", name.c_str());
        auto getParamOrError = [&](string name, auto& out) -> bool {
          if(!privateNodehandle.hasParam(name)) {
            ROS_ERROR("Error: the param '%s' was not found. Check if present and type", name.c_str());
            return false;
          }
          privateNodehandle.param(name, out, out);
          return true;
        };
        CurrentReaderConf conf;
        conf.name = name;
        if(!getParamOrError(Td::toString(name,"_port"), conf.port)) return false;
        if(!getParamOrError(Td::toString(name,"_v_to_amp_factor"), conf.vToAmpFactor)) return false;
        if(!getParamOrError(Td::toString(name,"_zero_offset"), conf. zeroOffset)) return false;

        if(privateNodehandle.hasParam(name+"_volt_range")) {
          string rangeStr;
          try {
            rangeStr = privateNodehandle.param<string>(Td::toString(name,"_volt_range"), rangeStr);
            conf.portVoltRange = voltageRangeToEnum(rangeStr, RangeType::bipolar);
          } catch(std::runtime_error& e) {
            ROS_ERROR("the value '%s' is not a valid voltage range", rangeStr.c_str());
            return false;
          }

        } else {
          ROS_ERROR("the param '%s' was not found. Check if present and type", Td::toString(name, "_volt_range").c_str());
          return false;
        }
        currentReader.addConfiguration(nodeHandle, conf);
      }
    }

    return true;
}


void IoAdcNode::run()
{
    ROS_INFO("Run");

    while(ros::ok()) {
        updateSuspensions();
        motorTempSensor.readNewValues(adcCard);
        motorTempSensor.publishNewValues();
        currentReader.update(adcCard);
        std::this_thread::sleep_for(sleepTime);
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

//     suspensionPublisher.publish(msg);
}

bool IoAdcNode::setEnableSuspensionPowerCallback(OnOffService::Request& req, OnOffService::Response& res)
{
    ros::NodeHandle privateNodehandle("~");
    int tepEnablePort; // TEP DC-DC converter enable port on IO card.
    privateNodehandle.param("suspension_power_enable_port", tepEnablePort, 4);

    uint16_t err;
    if((err = DO_WriteLine(ioCard, 0, tepEnablePort, req.on)) != NoError) {
      ROS_ERROR("Unable to write on IO card line %d", tepEnablePort);
      return false;
    } else {
      ROS_INFO("Suspension power enabled=%d", req.on);
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "io_adc");

    IoAdcNode ioAdcNode;

    if(!ioAdcNode.init()) {
        ROS_ERROR("error in init, exiting");
    }

    ioAdcNode.run();
}

