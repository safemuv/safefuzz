package atlascollectiveint.custom;

import atlascollectiveint.api.*;

import atlassharedclasses.SonarDetection;
import java.lang.Double;
import java.lang.String;
import java.util.HashMap;
import java.util.LinkedHashMap;

class ComputerCIshoreside {
	class PointCI {
		private double x;
		private double y;
		
		PointCI(double x,double y) {
			this.x=x;
			this.y=y;
		}
	}
	
	// The shoreside CI's copy of the robot locations
	private static HashMap<String,PointCI> robotLocations = new LinkedHashMap<String,PointCI>();
	private static HashMap<Integer,Integer> detectionCounts = new LinkedHashMap<Integer,Integer>(); 
	
  public static void init() {
	  
  }

  public static void SONARDetectionHook(SonarDetection detection, String robotName) {
	  // On a detection, if the detection is the first time...
	  // Send a second robot in to confirm
	  // Need to scan the positions to find the best choice
	  
	  // TODO: Need to move Point into atlassharedclasses
	  // and change all PointCI into Point
	  Point loc = detection.detectionLocation;
	  
	  
  }

  public static void GPS_POSITIONDetectionHook(Double x, Double y) {
	  // Update the robot position notification
	  // Need the robotname in here!
	  robotLocations.put(robotName, new PointCI(x,y));
  }
}
