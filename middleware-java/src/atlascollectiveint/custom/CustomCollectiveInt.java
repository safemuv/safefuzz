package atlascollectiveint.custom;

import atlassharedclasses.*;
import java.util.Optional;

import atlascollectiveintgenerator.CollectiveInt;
import atlassharedclasses.ATLASSharedResult;

public class CustomCollectiveInt extends CollectiveInt {
  protected void handleMessage(ATLASSharedResult a) {
    super.handleMessage(a);
    if (a.getContentsClass() == SonarDetection.class) {
    		  Optional<SonarDetection> d_o = a.getSonarDetection();
    		  if (d_o.isPresent()) {
    			  SonarDetection d = d_o.get();
    			  ComputerCIshoreside.SONARDetectionHook(d_o.get(), d.getRobotName());
    		  }
    	  }
  }

  public void init() {
    super.init();
    ComputerCIshoreside.init();
  }
}
