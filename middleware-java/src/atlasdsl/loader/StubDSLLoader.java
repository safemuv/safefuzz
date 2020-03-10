package atlasdsl.loader;

import java.util.Optional;
import atlasdsl.*;
import atlassharedclasses.Point;
import atlassharedclasses.Region;

public class StubDSLLoader implements DSLLoader {
	public static void addRobotWithSonar(Mission m, String robotName, Point startLocation, double startSpeed, double maxSpeed, int sensorRange, double detectionProb, double falsePos, double falseNeg) {
		Robot r = new Robot(robotName);
		Sensor sonar = new Sensor(SensorType.SONAR);
		Sensor gps = new Sensor(SensorType.GPS_POSITION);
		
		sonar.setIntComponentProperty("sensorRange", sensorRange);
		sonar.setIntComponentProperty("swathWidth", sensorRange);
		sonar.setDoubleComponentProperty("detectionProb", detectionProb);
		sonar.setDoubleComponentProperty("falsePos", falsePos);
		sonar.setDoubleComponentProperty("falseNeg", falseNeg);
		r.addSubcomponent(sonar);
		// TODO: properties for the GPS sensor?
		r.addSubcomponent(gps);	
		
		r.setPointComponentProperty("startLocation", startLocation);
		r.setPointComponentProperty("location", startLocation);
		
		r.setDoubleComponentProperty("startSpeed", startSpeed);
		r.setDoubleComponentProperty("maxSpeed", maxSpeed);
		m.addRobot(r);
	}
	
	public Mission loadMission() {
		// TODO: get these consistent with the values in the report object diagram
		Mission mission = new Mission();
		Computer shoreside = new Computer("shoreside");
		mission.addComputer(shoreside);
		
		addRobotWithSonar(mission, "gilda", new Point(0.0, 0.0),  1.0,  5.0, 50, 0.99, 0.01, 0.05);
		addRobotWithSonar(mission, "henry", new Point(40.0, 0.0), 1.5,  5.0, 25, 0.99, 0.03, 0.07);
		addRobotWithSonar(mission, "frank", new Point(60.0, 0.0), 1.0,  5.0, 10, 0.99, 0.03, 0.02);
		addRobotWithSonar(mission, "ella",  new Point(80.0, 0.0), 0.75, 5.0, 50, 0.99, 0.03, 0.06);
		
		// Add objects to the environment - hazards/benign objects for the robots to find
		mission.addObject(new EnvironmentalObject(new Point(10.0, -115.0), false));
		mission.addObject(new EnvironmentalObject(new Point(140.0,-65.0), true));
		mission.addObject(new EnvironmentalObject(new Point(135.0,-160.0), false));

		// Define the mission layer
		double MISSION_END_TIME = 1000.0;
		double AVOIDANCE_CLEARANCE = 3.0;
		
		Region fullRegion = new Region(new Point(-50.0,-230.0), new Point(200.0,-30.0));
		
		StaticGoalRegion staticRegion = new StaticGoalRegion(fullRegion);
		
		GoalTemporalConstraints entireMissionTime = new GoalTemporalConstraints(0.0, MISSION_END_TIME);
		GoalParticipants allRobots = (new StaticParticipants(StaticParticipants.Spec.ALL_ROBOTS, mission));
		Goal mutualAvoidance = new Goal("mutualAvoidance", mission, entireMissionTime, allRobots, Optional.empty(),	new AvoidOthers(AVOIDANCE_CLEARANCE));
		Goal primarySensorSweep = new Goal("primarySensorSweep", mission, entireMissionTime, allRobots, Optional.of(staticRegion), new CollectiveSensorCover(10.0, 1, SensorType.SONAR));
		
		RelativeParticipants rp = new RelativeParticipants(primarySensorSweep, ((StaticParticipants)allRobots), "DETECTION_UUV_NAME", RelativeParticipants.LogicOps.SUBTRACT, 1);
		
		double verifySweepRange = 30.0;
		
		// Need a concept of the goal action on the sensor detection and deactivation
		Goal verifySensor = new Goal("verifySensor", mission, entireMissionTime, rp, 
				Optional.of(new DynamicGoalRegion(primarySensorSweep, "detectionCoord", verifySweepRange)),
				new SensorCover(20.0, SensorType.SONAR));
		
		mission.addGoal("mutualAvoidance", mutualAvoidance);
		mission.addGoal("primarySensorSweep", primarySensorSweep);
		mission.addGoal("verifySensor", verifySensor);
		
		// Add more message definitions here
		mission.addMessage(new Message("detectionGilda", shoreside, mission.getRobot("gilda")));
		mission.addMessage(new Message("detectionHenry", shoreside, mission.getRobot("henry")));
		mission.addMessage(new Message("detectionFrank", shoreside, mission.getRobot("frank")));
		mission.addMessage(new Message("detectionElla",  shoreside, mission.getRobot("ella")));
		return mission;
	}
}
