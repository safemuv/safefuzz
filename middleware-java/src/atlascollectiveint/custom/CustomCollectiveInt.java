package atlascollectiveint.custom;

import java.util.Optional;

import atlascollectiveintgenerator.CollectiveInt;
import atlassharedclasses.*;

public class CustomCollectiveInt extends CollectiveInt {
  void handleMessage(ATLASSharedResult a) {
    if (a.getContentsClass() == SonarDetection.class) {
    		  Optional<SonarDetection> d_o = a.getSonarDetection();
    		  if (d_o.isPresent()) {
    			  SonarDetection d = d_o.get();
    			  ComputerCIshoreside.SONARDetectionHook(d_o.get(), d.getRobotName());
    		  }
    	  }if (a.getContentsClass() == GPSPositionReading.class) {
    		  Optional<GPSPositionReading> r_o = a.getGPSPositionReading();
    		  if (r_o.isPresent()) {
    			  GPSPositionReading r = r_o.get();
    			  ComputerCIshoreside.GPS_POSITIONDetectionHook(r.getX(),r.getY());
    		  }
    	  }
  }
}
