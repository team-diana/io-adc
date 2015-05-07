#include "io_adc/io_adc_node.hpp"
#include "io_adc/dask.h"
#include "io_adc/utils.hpp"

#include "io_adc/sosp_Adc.h"

using namespace io_adc;

IoAdcNode::IoAdcNode() : nodeHandle("io_adc")
{
    ros::NodeHandle private_node_handle("~");
//     private_node_handle.param("message", message, std::string("NO error"));
//     private_node_handle.param("rate", rate, int(RATE));


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
    if ((err = AI_9116_Config(adcCard, P9116_AI_SingEnded|P9116_AI_UserCMMD, P9116_AI_SoftPolling ,0, 0, 0)) != NoError) {
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


int main(int argc, char** argv) {
  ros::init(argc, argv, "io_adc");

  IoAdcNode ioAdcNode;

  if(!ioAdcNode.init()) {
    ROS_ERROR("error in init, exiting");
  }

  ioAdcNode.run();
}

