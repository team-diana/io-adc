#include "io_adc/io_scope_node.hpp"
#include "io_adc/utils.hpp"
#include "io_adc/dask.h"
#include "io_adc/consts.hpp"
#include "io_adc/io_scope.h"

#include <team_diana_lib/strings/strings.h>

#include <algorithm>

using namespace io_adc;

IoScopeNode::IoScopeNode() : nodeHandle("io_scope")
{
  uint32_t i = 0;
  scopePublisher = nodeHandle.advertise<io_adc::io_scope>("io_scope", 100);
//   ros::NodeHandle private_node_handle("~");
//   private_node_handle.param(Td::toString("voltage_range_", i), voltageRange, 2.5);
//   private_node_handle.param(Td::toString("voltage_range_", i, "_unipolar"), unipolar, false);
}

IoScopeNode::~IoScopeNode()
{
  ROS_INFO("Releasing cards");

  Release_Card(ioCard);
}


bool IoScopeNode::init()
{
    uint16_t err;

    ROS_INFO("Init PCI_7432");

    if ((ioCard = Register_Card(PCI_7432, 0)) < 0) {
      log_register_card_error("pCI_7432", ioCard);
      return false;
    }

    ROS_INFO("Init done");

    return true;
}


void IoScopeNode::run()
{
  ROS_INFO("Run");

  while(ros::ok()) {

    // Note, actually it is supposed to be uint32_t, but the library has an error
    uint64_t portValue;
    uint16_t err;
    if((err = DO_ReadPort(ioCard, 0, &portValue)) != NoError) {
      log_input_error_dio(0, " while reading port", err);
    }
   
    std::vector<uint8_t> lines;

    // From MSB to LSB
    for(int i = 31; i >= 0; i--) {
      // TODO: check that this is actually right.
      bool lineOn = portValue & (1 << i);
      lines.push_back(lineOn);
    }

    io_scope msg;
    msg.port = portValue;
    msg.lines = lines;

    scopePublisher.publish(msg);

    ros::spinOnce();
  }

  ROS_INFO("Stop");
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "io_scope");

  IoScopeNode ioScopeNode;
  if(ioScopeNode.init()) {
    ROS_ERROR("unable to open IO card, exiting");
    return -1;
  }

  ioScopeNode.run();
}
