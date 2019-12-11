#include "MOOS/libMOOS/DB/MOOSDB_ATLAS.h"
#include "MOOS/libMOOS/DB/MOOSDB.h"

#include <iostream>
#include <sstream>

#include "MOOS/libMOOS/DB/ATLASLinkProducer.h"
#include "MOOS/libMOOS/DB/ATLASLinkConsumer.h"

using namespace std;

CMOOSDB_ATLAS::CMOOSDB_ATLAS(int port, string mission_file) {
    debug_output.open("/tmp/af_debug" + mission_file + ".log");
    debug_output << "MOOSDB_ATLAS debug" << endl;

    activeMQPort = port;
    this->mission_file = mission_file;

    ostringstream ss;
    ss << brokerURIBase << to_string(port) << ")";

    broker_uri = ss.str();
    cout << "Creating CMOOSDB_ATLAS... activating ActiveMQ interface on "
         << port << endl;
    cout << "Connecting to " << broker_uri << endl;

    startMQInterface();
    prod = new ATLASLinkProducer(this, broker_uri, mission_file);
    cons = new ATLASLinkConsumer(this, broker_uri, mission_file);
    cout << "ActiveMQ connection completed!" << endl;
}

/*
bool CMOOSDB_ATLAS::OnNotify(CMOOSMsg &Msg) {
  cout << "CMOOSDB_ATLAS::OnNotify" << endl;
  double dfTimeNow = HPMOOSTime();
  CMOOSDBVar &rVar = GetOrMakeVar(Msg);

  // If problems, check the value of this variable, in
  // cases in which overrides are still in effect
  bool res = false;

  // If we have not overriden this variable,
  // override time will be less than the current time.
  // This is always true since it begins as -1.0
  if (rVar.m_dfOverrideTime < dfTimeNow) {
    // In this case, handle notification normally
    // by calling the superclass method
    res = CMOOSDB::OnNotify(Msg);
    // Reset the override
    rVar.m_dfOverrideTime = -1.0;
  } else {
    // Override is still in the future
    cout << "Ignored msg due to override" << endl;
  }
  return res;
}
*/

bool CMOOSDB_ATLAS::fromMQ(CMOOSMsg &Msg) {
    cout << "Message over ActiveMQ: no time registered" << endl;
    // When a message notification comes in from the ActiveMQ,
    // always handle it as a standard notification
    return CMOOSDB::OnNotify(Msg);
}

bool CMOOSDB_ATLAS::fromMQ(CMOOSMsg &Msg, double overrideTimeEnd) {
    cout << "Messge over ActiveMQ: overrideTimeEnd = " << overrideTimeEnd << endl;
    CMOOSDBVar &rVar = GetOrMakeVar(Msg);
    // When a message notification comes in from the ActiveMQ,
    // always handle it as a standard notification
    bool res = CMOOSDB::OnNotify(Msg);
    // Need to set an override on this message
    // rVar.m_dfOverrideTime = overrideTimeEnd;
    // Return the original result from the notify call
    return res;
}

void CMOOSDB_ATLAS::startMQInterface() {
  activemq::library::ActiveMQCPP::initializeLibrary();
}

void CMOOSDB_ATLAS::stopMQInterface() {
  activemq::library::ActiveMQCPP::shutdownLibrary();
}
