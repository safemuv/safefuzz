package atlascollectiveint.custom;

import atlascollectiveint.api.*;

import atlassharedclasses.*;

import java.lang.Double;
import java.lang.String;
import java.util.HashMap;
import java.util.LinkedHashMap;

class ComputerCIshoreside {

	// The shoreside CI's copy of the robot locations
	private static HashMap<String,Point> robotLocations = new LinkedHashMap<String,Point>();
	private static HashMap<Integer,Integer> detectionCounts = new LinkedHashMap<Integer,Integer>();

    private static void freshDetection(int label) {
        if ((c = detectionCounts.get(label)) == null) {
            detectionCounts.put(label,0);
            return true;
        } else {
            detectionCounts.put(label,c+1);
            return false;
        }
    }
    
  public static void init() {
      // setup the robot definitions - or should this be part of generated code?
      // divide up the rect region amongst the robots
      // set their original behaviour sweeps on a stack
  }

  public static void SONARDetectionHook(SonarDetection detection, String robotName) {
	  // On a detection, if the detection is the first time...
	  // Send a second robot in to confirm
	  // Need to scan the positions to find the best choice

	  // TODO: Need to move Point into atlassharedclasses
	  // and change all PointCI into Point
	  Point loc = detection.detectionLocation;
    int label = detection.objectID;

    // Count the detections at this location
    if (freshDetection(label)) {
        // choose robot
        String rName = findRobotNear(loc);
        Behaviour.setRegionSweep(rName, loc, RADIUS);
        // need to send this robot back to its original action after some time...
        // TODO: use a timer here to pop from the robot action stack
        // will need to track the robot active regions on some sort of per-robot stack?
    }
  }

  public static void GPS_POSITIONDetectionHook(Double x, Double y) {
	  // Update the robot position notification
	  // TODO: Need the robotname in this hook!
	  robotLocations.put(robotName, new Point(x,y));
  }
}
