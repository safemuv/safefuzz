package atlascollectiveint.custom.backup;

import atlascollectiveint.api.*;
import atlascollectiveint.logging.CollectiveIntLog;
import atlasdsl.Goal;
import atlasdsl.GoalAction;
import atlasdsl.GoalRegion;
import atlasdsl.MissingProperty;
import atlasdsl.Mission;
import atlasdsl.Robot;
import atlasdsl.Sensor;
import atlasdsl.SensorCover;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
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

class ComputerCIshoreside_advanced {

	private static Mission mission;

	// The shoreside copy of the robot information
	private static List<String> sweepRobots = new ArrayList<String>();
	private static List<String> cameraRobots = new ArrayList<String>();
	private static List<String> allRobots = new ArrayList<String>();

	private static HashMap<String,Double> robotSensorWidths = new LinkedHashMap<String, Double>();
	private static HashMap<String, Point> robotLocations = new LinkedHashMap<String, Point>();
	private static HashMap<Integer, Integer> detectionCounts = new LinkedHashMap<Integer, Integer>();
	private static HashMap<String, Boolean> robotIsConfirming = new LinkedHashMap<String, Boolean>();
	private static HashMap<String, Region> robotSweepRegions = new LinkedHashMap<String, Region>();
	private static HashMap<String, Double> robotSpeeds = new LinkedHashMap<String, Double>();
	
	
	private static final Region DEFAULT_REGION = new Region(new Point(-50.0, -230.0), new Point(200.0, -30.0));
	private static Region fullRegion = DEFAULT_REGION;
	
	private static int numberOfVerificationsMalicious;
	private static int numberOfVerificationsBenign;
	
	private static final double SWEEP_RADIUS = 50.0;
	private static final double VERTICAL_STEP_SIZE_INITIAL_SWEEP = 30;
	private static final double VERTICAL_STEP_SIZE_CONFIRM_SWEEP = 10;
	private static final int VERTICAL_ROWS_STATIC_SPLIT = 1;

	private static final double TIME_SPENT_VERIFYING = 500.0;
	private static final double CAMERA_DIVE_DEPTH = 20.0;

	private static boolean freshDetection(int label) {
		Integer c = detectionCounts.get(label);
		if (c == null) {
			detectionCounts.put(label, 0);
			return true;
		} else {
			detectionCounts.put(label, c + 1);
			return false;
		}
	}

	private static Map<String, Double> robotDistancesTo(Point loc) {
		Map<String, Double> res;
		res = robotLocations.entrySet().stream()
				.collect(Collectors.toMap(e -> e.getKey(), e -> loc.distanceTo(e.getValue())));
		return res;
	}

	private static double travelTimeForRobot(String robotName, double dist) {
		Double speed = robotSpeeds.get(robotName);
		if (speed == null) {
			System.out.println(
					"Robot " + robotName + " does not have a speed defined: setting its travel time to maximum");
			return Double.MAX_VALUE;
		} else {
			double time = dist / speed;
			return time;
		}
	}

	// The shoreside chooses a robot to use to confirm detections,
	// excluding the detecting one obviously!
	private static Optional<String> chooseRobot(Point loc, List<String> allowedRobots, String excludeRobot) {
		Map<String, Double> dists = robotDistancesTo(loc);
		Optional<Map.Entry<String, Double>> res = dists.entrySet().stream()
				// Check it isn't the exclude robot
				.filter(e -> e.getKey().compareTo(excludeRobot) != 0 && allowedRobots.contains(e.getKey()))
				// Check robot is not already confirming!
				.filter(e -> !robotIsConfirming.get(e.getKey()))
				// Have to transform to new Map - here the distance is mapped to the travel time
				.collect(Collectors.toMap(Map.Entry::getKey, e -> travelTimeForRobot(e.getKey(), e.getValue())))
				.entrySet().stream().sorted(Map.Entry.comparingByValue()).findFirst();
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
	
	public static List<String> getAllRobotsByDepth(boolean onSurface) throws MissingProperty {
		List<String> rmap = new ArrayList<String>();
		for (Robot robot : mission.getAllRobots()) {
			if ((robot.getDoubleComponentProperty("maxDepth") < 1e6) == onSurface) {
				rmap.add(robot.getName());
			}
		}
		return rmap;
	}

	public static void setSystemStateFromModel() throws DSLLoadFailed, MissingProperty {
		loadDSL();
		robotSpeeds = getRobotNamesAndSpeeds();
		robotSensorWidths = getRobotSensorWidths();
		
		// The sweep robots are the surface-only robots
		sweepRobots = getAllRobotsByDepth(true);
		// The camera robots are the depth-capable robots
		cameraRobots = getAllRobotsByDepth(false);

		// Setup robot state
		for (String r : sweepRobots) {
			robotIsConfirming.put(r, false);
			allRobots.add(r);
		}

		for (String r : cameraRobots) {
			robotIsConfirming.put(r, false);
			allRobots.add(r);
		}
		
		// TODO: how to modify the model to specify this for benign vs malicious?
		// And broken down by the type of the vehicle needed

		Goal sensorSweep = mission.getGoalByName("primarySensorSweep");
		GoalAction sensorSweepAct = sensorSweep.getAction();
		Goal sensorVerify = mission.getGoalByName("verifySensorDetections"); 
		GoalAction sensorVerifyAct = sensorVerify.getAction();
		
		if (sensorVerifyAct instanceof SensorCover) {
			SensorCover sc = (SensorCover)sensorVerifyAct;
			numberOfVerificationsMalicious = sc.verificationsMalicious();
			numberOfVerificationsBenign = sc.verificationsBenign();
		}
		
		Optional<GoalRegion> r_o = sensorSweep.getGoalRegion();
		if (r_o.isPresent()) {
				r_o.get();
				GoalRegion go = r_o.get();
				fullRegion = go.getFirstRegion().get();
		}
	}

	/** Returns the region size proportion from the speeds */
	public static Map<String, Double> regionProportionsFromSpeeds(List<String> robots) {
		HashMap<String, Double> regionProportions = new HashMap<String, Double>();

		double totalSpeed = 0.0;
		// Sum the total speed for the given robots
		for (Map.Entry<String, Double> rs : robotSpeeds.entrySet()) {
			// only count the named robots
			if (robots.contains(rs.getKey())) {
				totalSpeed += rs.getValue();
			}
		}

		//
		for (Map.Entry<String, Double> rs : robotSpeeds.entrySet()) {
			// only count the named robots
			if (robots.contains(rs.getKey())) {
				double rprop = rs.getValue() / totalSpeed;
				regionProportions.put(rs.getKey(), rprop);
				System.out.println("regionProportion for " + rs.getKey() + "=" + rprop);
			}
		}

		return regionProportions;
	}

	// Just try setting up the regions here as a constant strip
	public static Map<String, Region> staticRegionSplitBySpeeds(Region fullRegion, List<String> robots) {
		HashMap<String, Region> assignments = new HashMap<String, Region>();

		int count = robots.size();
		int hcount = count;
		int vcount = VERTICAL_ROWS_STATIC_SPLIT;
		double subheight = fullRegion.height() / vcount;

		int xindex = 0;
		double xl = fullRegion.left();
		double yb = fullRegion.bottom();
		for (Map.Entry<String, Double> rs : regionProportionsFromSpeeds(robots).entrySet()) {
			double subwidth = fullRegion.width() * rs.getValue() * vcount;
			Region subr = new Region(xl, yb, xl + subwidth, yb + subheight);
			// Need to know the total proportion taken up on a particular row,
			// then scale the height according to this!
			xl += subwidth;
			xindex++;
			if (xindex == hcount) {
				xindex = 0;
				yb += subheight;
				xl = fullRegion.left();
			}
			assignments.put(rs.getKey(), subr);
		}
		return assignments;
	}

	public static void loadDSL() throws DSLLoadFailed {
		DSLLoader dslloader = new GeneratedDSLLoader();
		mission = dslloader.loadMission();
		// Things to load from the database:
		// the number of robots and their speed properties
	}

	public static HashMap<String, Double> getRobotNamesAndSpeeds() throws MissingProperty {
		HashMap<String, Double> rSpeeds = new HashMap<String, Double>();
		for (Robot robot : mission.getAllRobots()) {
			double speed = robot.getDoubleComponentProperty("startSpeed");
			rSpeeds.put(robot.getName(), speed);
		}
		return rSpeeds;
	}
	
	public static HashMap<String, Double> getRobotSensorWidths() throws MissingProperty {
		HashMap<String, Double> rSensorWidths = new HashMap<String, Double>();
		for (Robot robot : mission.getAllRobots()) {
			Optional<Sensor> s_o = robot.getFirstSensor();
			
			// Assume a default width if there is nothing defined
			double width = VERTICAL_STEP_SIZE_INITIAL_SWEEP;
			if (s_o.isPresent()) {
				Sensor s = s_o.get();
				width = s.getDoubleComponentProperty("swathWidth");
			} else {
				System.out.println("WARNING: Sensor not found on vehicle " + robot.getName() + " - assuming default width");
			}
				
			System.out.println("Robot " + robot.getName() + " - sensor width=" + width);
			rSensorWidths.put(robot.getName(), width);
		}
		return rSensorWidths;
	}

	public static void init() {
		try {
			System.out.println("init");
			// Load the DSL contents, and load some information from the vehicles
			setSystemStateFromModel();
			System.out.println("robotSpeeds = " + robotSpeeds);
			System.out.println("fullRegion = " + fullRegion);

			Map<String, Region> regionAssignments = staticRegionSplitBySpeeds(fullRegion, sweepRobots);
			CollectiveIntLog.logCI("ComputerCIshoreside.init - regionAssignments length = " + regionAssignments.size());

			// Start all the robots first
			for (Map.Entry<String, Region> e : regionAssignments.entrySet()) {
				String robot = e.getKey();
				Region region = e.getValue();
				CollectiveIntLog.logCI("Starting sweep robot " + robot);
				API.startVehicle(robot);
			}

			// Then set the region assignments
			for (Map.Entry<String, Region> e : regionAssignments.entrySet()) {
				String robot = e.getKey();
				Region region = e.getValue();
				Double sensorWidth = robotSensorWidths.get(robot);
				if (sensorWidth == null) {
					sensorWidth = VERTICAL_STEP_SIZE_INITIAL_SWEEP;
				}
				
				API.setPatrolAroundRegion(robot, region, sensorWidth,
						("UUV_COORDINATE_UPDATE_INIITAL_" + robot.toUpperCase()));

				CollectiveIntLog.logCI("Setting robot " + robot + " to scan region " + region.toString());
				robotSweepRegions.put(robot, region);
			}

			for (String robot : cameraRobots) {
				CollectiveIntLog.logCI("Starting camera robot " + robot);
				API.startVehicle(robot);
				API.setDepth(robot, CAMERA_DIVE_DEPTH, "DEPTH_SET_MESSAGE_" + robot);
				CollectiveIntLog.logCI("Starting camera robot " + robot);
			}
			
		} catch (DSLLoadFailed e1) {
			CollectiveIntLog.logCI("DSL Loading Failed");
			e1.printStackTrace();
		} catch (MissingProperty e1) {
			CollectiveIntLog.logCI("Missing property during processing");
			e1.printStackTrace();
		}
	}

	public static void verifyOneRobot(SensorDetection detection, List<String> candidateRobots,
			String detectingRobotName) {
		Point loc = (Point) detection.getField("location");
		Optional<String> rName_o = chooseRobot(loc, candidateRobots, detectingRobotName);
		if (rName_o.isPresent()) {
			String rName = rName_o.get();
			API.setSweepAroundPoint(rName, loc, SWEEP_RADIUS, VERTICAL_STEP_SIZE_CONFIRM_SWEEP,
					("UUV_COORDINATE_UPDATE_VERIFY_" + rName.toUpperCase()));
			CollectiveIntLog.logCI("Setting robot " + rName + " to verify detection");

			// Need to send this robot back to its original action after some time...
			// Register a one-off timer to return the robot to its original activity
			OneOffTimer treturn = OneOffTimer.afterDelay(TIME_SPENT_VERIFYING, (t -> {
				CollectiveIntLog.logCI("Verification timer expired for robot " + rName);
				Region origRegion = robotSweepRegions.get(rName);
				if (origRegion != null) {
					// Set the robot as available for future detections
					robotIsConfirming.put(rName, false);
					CollectiveIntLog.logCI("Setting robot " + rName + " to return to region " + origRegion.toString());
					API.setPatrolAroundRegion(rName, origRegion, VERTICAL_STEP_SIZE_INITIAL_SWEEP,
							("UUV_COORDINATE_UPDATE_INIITAL_" + rName.toUpperCase()));
				}
			}));

			API.registerTimer(rName, treturn);

		} else {
			CollectiveIntLog.logCI("ERROR: No robots avaiable to confirm the detection");
		}
	}

	public static void SONARDetectionHook(SensorDetection detection, String robotName) {
		int label = (Integer) detection.getField("objectID");
		String detectionType = (String) detection.getField("type");

		if (freshDetection(label)) {
			if (detectionType.equals("benign")) {
				for (int i = 0; i < numberOfVerificationsBenign; i++) {
					verifyOneRobot(detection, sweepRobots, robotName);
				}
			} else {
				verifyOneRobot(detection, cameraRobots, robotName);
				int numV = Math.max(numberOfVerificationsMalicious, 0);
				for (int i = 0; i < numV; i++) {
					verifyOneRobot(detection, sweepRobots, robotName);
				}
			}
		}
	}

	public static void GPS_POSITIONDetectionHook(Double x, Double y, String robotName) {
		// Update the robot position notification
		robotLocations.put(robotName, new Point(x, y));
	}

	public static void CAMERADetectionHook(SensorDetection detection, String robotName) {
		Point loc = (Point) detection.getField("location");
		int objectID = (Integer) detection.getField("objectID");
		String detectionType = (String) detection.getField("type");
		CollectiveIntLog.logCI(
				"Camera detection of object ID " + objectID + "at location" + " type " + detectionType + robotName);
	}
}