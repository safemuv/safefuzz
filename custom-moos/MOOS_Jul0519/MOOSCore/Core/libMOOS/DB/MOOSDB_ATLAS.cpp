#include "MOOS/libMOOS/DB/MOOSDB_ATLAS.h"
#include "MOOS/libMOOS/DB/MOOSDB.h"

#include <iostream>
#include <sstream>

#include "MOOS/libMOOS/DB/ATLASLinkProducer.h"
#include "MOOS/libMOOS/DB/ATLASLinkConsumer.h"

using namespace std;

CMOOSDB_ATLAS::CMOOSDB_ATLAS(int port, string mission_file) {
    starting_moos_time = HPMOOSTime();
    cout << "starting_moos_time = " << starting_moos_time << endl;
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

bool CMOOSDB_ATLAS::faultInEffect(CMOOSDBVar &rVar) {
        double dfTimeNow = HPMOOSTime() - starting_moos_time;
        cout << "timeNow = " << dfTimeNow << endl;
        return (rVar.m_dfOverrideTime > dfTimeNow);
}

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
    // but first reset the time to zero
    rVar.m_dfOverrideTime = 0.0;
    bool res = CMOOSDB::OnNotify(Msg);
    // Need to set an override on this message
    rVar.m_dfOverrideTime = overrideTimeEnd;

    CMOOSDBVar &rVar2 = GetOrMakeVar(Msg);
    cout << "rVar.m_dfOverrideTime set to " << rVar2.m_dfOverrideTime << endl;
    return res;
}

void CMOOSDB_ATLAS::startMQInterface() {
  activemq::library::ActiveMQCPP::initializeLibrary();
}

void CMOOSDB_ATLAS::stopMQInterface() {
  activemq::library::ActiveMQCPP::shutdownLibrary();
}
