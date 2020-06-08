#pragma once

#include <functional>

#include "ATLASLinkApp.h"
#include "MOOS/libMOOS/MOOSLib.h"

class ATLASLinkConsumer;

class ATLASDBInterface : public CMOOSApp {
public:
  ATLASDBInterface();
  virtual ~ATLASDBInterface() {}

  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  void fromMQHook(CMOOSMsg &msg, double endTime);

protected:
  ATLASLinkProducer *prod;
  ATLASLinkConsumer *cons;
  void RegisterVariables();
  bool SetupActiveMQ();
  void ProcessMissionFile();
  void SecondaryProcessMissionFile();
  bool ScanForVariable(const string &fileLine, const string &targetVar, function <void(string)> matchAction);
  vector<string> vars_to_watch;
  int mq_activemq_port = 0;
  string mq_topic_name_prod = "PROD";
  string mq_topic_name_cons = "CONS";
};
