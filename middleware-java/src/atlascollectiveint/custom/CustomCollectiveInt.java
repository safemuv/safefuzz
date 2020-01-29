package collectiveint.user;

import atlascollectiveintgenerator.CollectiveInt;
import middleware.core.SensorDetection;

class CustomCollectiveInt extends CollectiveInt {
  void handleDetction(SensorDetection d) {
    if (d.getSensorType() == SensorType.SONAR) {
    ComputerCIShoreside.SONARDetectionHook(d,d.getRobot());
    }
    if (d.getSensorType() == SensorType.GPS_POSITION) {
    ComputerCIShoreside.GPS_POSITIONDetectionHook(d,d.getRobot());
    }
  }
}
