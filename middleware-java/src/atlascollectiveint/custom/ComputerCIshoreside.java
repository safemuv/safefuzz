package atlascollectiveint.custom;

import atlassharedclasses.SonarDetection;
import java.lang.Double;
import java.lang.String;

class ComputerCIshoreside {
  public static void init() {
  }

  public static void SONARDetectionHook(SonarDetection detection, String robotName) {
	  System.out.println("DETECTION");
  }

  public static void GPS_POSITIONDetectionHook(Double x, Double y) {
	  System.out.println("GPS at CI");
  }
}
