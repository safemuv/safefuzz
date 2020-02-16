package atlascollectiveint.custom;

import atlascollectiveint.api.*;
import atlascollectiveint.logging.CollectiveIntLog;
import atlassharedclasses.*;

import java.lang.Double;
import java.lang.String;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

class ComputerCIshoreside {

	// The shoreside CI's copy of the robot information
	private static List<String> robots = new ArrayList<String>();
	private static HashMap<String,Point> robotLocations = new LinkedHashMap<String,Point>();
	private static HashMap<Integer,Integer> detectionCounts = new LinkedHashMap<Integer,Integer>();
	private static HashMap<String,Boolean> robotIsConfirming = new LinkedHashMap<String,Boolean>();
	private static Region fullRegion;

	private static final double SWEEP_RADIUS = 50.0;
	private static final double VERTICAL_STEP_SIZE_INITIAL_SWEEP = 30;
	private static final double VERTICAL_STEP_SIZE_CONFIRM_SWEEP = 10;
	private static final int VERTICAL_ROWS_STATIC_SPLIT = 2;
	
    private static boolean freshDetection(int label) {
    	Integer c = detectionCounts.get(label);
        if (c == null) {
            detectionCounts.put(label,0);
            return true;
        } else {
            detectionCounts.put(label,c+1);
            return false;
        }
    }
    
    private static Map<String,Double> robotDistancesTo(Point loc) {
    	Map<String,Double> res;
    	res = robotLocations.entrySet().stream().
    			collect(Collectors.toMap(
    	            e -> e.getKey(),
    	            e -> loc.distanceTo(e.getValue())));
    	return res;
    }
    
    // The shoreside chooses a robot to use to confirm detections, 
    // excluding the detecting one obviously!
    private static Optional<String> chooseRobotNear(Point loc, String excludeRobot) {

    	Map<String,Double> dists = robotDistancesTo(loc);
    	Optional<Map.Entry<String, Double>> res = dists.entrySet().stream()
    			// Check it isn't the exclude robot
    			.filter(e -> e.getKey().compareTo(excludeRobot) != 0)
    			// Check robot is not already confirming!
    			.filter(e -> !robotIsConfirming.get(e.getKey()))
    			.sorted(Map.Entry.comparingByValue())
    			.findFirst();
    	// TODO: check sort order here
    	
    	if (res.isPresent()) {
    		// If a robot found, set it as confirming a detection
    		String chosen = res.get().getKey();
    		robotIsConfirming.put(chosen, true);
    		return Optional.of(res.get().getKey());
    	} else {
    		// If no robots are known, it will be empty  
    		return Optional.empty();
    	}    	
    }
    
  public static void setRobotNamesAndRegion() {
	  // TODO: should we get this from the DSL info?
	  robots.add("frank");
	  robots.add("gilda");
	  robots.add("henry");
	  robots.add("ella");
	  
	  for (String r : robots) {
		  robotIsConfirming.put(r, false);
	  }
		  
	  fullRegion = new Region(new Point(-50.0,-230.0), new Point(200.0,-30.0));
  }
  
  public static Map<String,Region> staticRegionSplit(Region fullRegion, List<String> robots) {
	  HashMap<String,Region> assignments = new HashMap<String,Region>();
	  
	  int count = robots.size();
	  int hcount = count / 2;
	  int vcount = VERTICAL_ROWS_STATIC_SPLIT;
	  double subwidth = fullRegion.width() / hcount;
	  double subheight = fullRegion.height() / vcount;
	  for (int i=0;i<count;i++) {
		  String robot = robots.get(i);
		  // TODO: fix these expressions
		  double xl = fullRegion.left() + i % hcount * subwidth;
		  double yb = fullRegion.bottom() + i / vcount * subheight;
		  Region subr = new Region(xl,yb,xl+subwidth, yb+subheight);
		  assignments.put(robot, subr);
	  }
	
	  return assignments;	  
  }
    
  public static void init() {
	  System.out.println("init");
      // TODO: load the robot definition names - or should this be part of generated code?
	  // for now, hardcode it statically
	  setRobotNamesAndRegion();
	  Map<String,Region> regionAssignments = staticRegionSplit(fullRegion,robots);
	  CollectiveIntLog.logCI("ComputerCIshoreside.init - regionAssignments length = " + regionAssignments.size());
	  for (Map.Entry<String, Region> e : regionAssignments.entrySet()) {
		  String robot = e.getKey();
		  Region region = e.getValue();
		  RobotBehaviours.setPatrolAroundRegion(robot, region, VERTICAL_STEP_SIZE_INITIAL_SWEEP);
		  CollectiveIntLog.logCI("Setting robot " + robot + " to scan region " + region.toString());
	  }
	  
      // divide up the rect region amongst the robots
      // set their original behaviour sweeps on a stack?
	  // look in the lisp version to see how this is done
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
        // choose robot to do the confirmation
        Optional<String> rName_o = chooseRobotNear(loc, robotName);
        if (rName_o.isPresent()) {
        	String rName = rName_o.get();
            RobotBehaviours.setSweepAroundPoint(rName, loc, SWEEP_RADIUS, VERTICAL_STEP_SIZE_CONFIRM_SWEEP);
            CollectiveIntLog.logCI("Setting robot " + rName + " to confirm sweep");
            // need to send this robot back to its original action after some time...
            // TODO: use a timer here to pop from the robot action stack
            // will need to track the robot active regions on some sort of per-robot stack?
        } else {
        	CollectiveIntLog.logCI("ERROR: No other robots avaiable to confirm the detection");
        }
    }
  }

  public static void GPS_POSITIONDetectionHook(Double x, Double y, String robotName) {
	  // Update the robot position notification
	  // TODO: Need the robotname in this hook in the code generator!
	  robotLocations.put(robotName, new Point(x,y));
  }
}