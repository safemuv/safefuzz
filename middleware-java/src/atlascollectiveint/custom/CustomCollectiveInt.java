package atlascollectiveint.custom;

import atlassharedclasses.*;
import java.util.Optional;

import atlascollectiveintgenerator.CollectiveInt;
import atlassharedclasses.ATLASSharedResult;

public class CustomCollectiveInt extends CollectiveInt {
  protected void handleMessage(ATLASSharedResult a) {
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
    			  ComputerCIshoreside.GPS_POSITIONDetectionHook(r.getX(),r.getY(), r.getRobotName());
    		  }
    	  }
  }

  public void init() {
	  System.out.println("CustomCollectiveInt.init() called");
    ComputerCIshoreside.init();
  }
}
