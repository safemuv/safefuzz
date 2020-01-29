package collectiveint.user;

import java.util.Optional;

import atlascollectiveintgenerator.CollectiveInt;
import atlassharedclasses.ATLASSharedResult;
import atlassharedclasses.SonarDetection;

class CustomCollectiveIntExample extends CollectiveInt {
  void handleMessage(ATLASSharedResult a) {
	  if (a.getContentsClass() == SonarDetection.class) {
		  Optional<SonarDetection> d_o = a.getSonarDetection();
		  if (d_o.isPresent()) {
			  SonarDetection d = d_o.get();
			  ComputerCIshoreside.SONARDetectionHook(d_o.get(), d.getRobotName());
		  }
	  }
	  
	  
		  
		  
    //if (d.getSensorType() == SensorType.SONAR) {
    //ComputerCIShoreside.SONARDetectionHook(d,d.getRobot());
    //}
    //if (d.getSensorType() == SensorType.GPS_POSITION) {
    //ComputerCIShoreside.GPS_POSITIONDetectionHook(d,d.getRobot());
    //}
  }
}