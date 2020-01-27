package atlasdsl.loader;

import java.util.Optional;

import atlasdsl.*;

public class StubDSLLoader implements DSLLoader {
	public static void addRobotWithSonar(Mission m, String robotName, Point startLocation, int sensorRange, double detectionProb, double falsePos, double falseNeg) {
		Robot r = new Robot(robotName);
		Sensor s = new Sensor(SensorType.SONAR);
		s.setIntComponentProperty("sensorRange", sensorRange);
		s.setIntComponentProperty("swathWidth", sensorRange);
		s.setDoubleComponentProperty("detectionProb", detectionProb);
		s.setDoubleComponentProperty("falsePos", falsePos);
		s.setDoubleComponentProperty("falseNeg", falseNeg);
		r.addSubcomponent(s);
		r.setPointComponentProperty("startLocation", startLocation);
		r.setPointComponentProperty("location", startLocation);
		m.addRobot(r);
	}
	
	public Mission loadMission() {
		// TODO: get these consistent with the values in the report object diagram
		Mission mission = new Mission();
		Computer shoreside = new Computer("shoreside");
		mission.addComputer(shoreside);
		
		addRobotWithSonar(mission, "gilda", new Point(0.0, 0.0), 50, 0.9, 0.01, 0.05);
		addRobotWithSonar(mission, "henry", new Point(10.0, 0.0), 50, 0.8, 0.03, 0.07);
		addRobotWithSonar(mission, "frank", new Point(20.0, 0.0), 50, 0.2, 0.03, 0.02);
		addRobotWithSonar(mission, "ella",  new Point(30.0, 0.0), 50, 0.2, 0.03, 0.06);
		
		// Add objects to the environment - hazards/benign objects for the robots to find
		mission.addObject(new EnvironmentalObject(new Point(46.0, -23.0), false));
		mission.addObject(new EnvironmentalObject(new Point(36.0, -13.0), true));
		mission.addObject(new EnvironmentalObject(new Point(66.0, -3.0), false));

		// Define the mission layer
		double MISSION_END_TIME = 1000.0;
		
		GoalTemporalConstraints entireMissionTime = new GoalTemporalConstraints(0.0, MISSION_END_TIME);
		GoalParticipants allRobots = (new StaticParticipants(StaticParticipants.Spec.ALL_ROBOTS, mission));
		Goal mutualAvoidance = new Goal(entireMissionTime, allRobots, Optional.empty(),	new AvoidOthers());
		Goal primarySensorSweep = new Goal(entireMissionTime, allRobots, Optional.empty(), new CollectiveSensorCover(10.0, SensorType.SONAR));
		
		RelativeParticipants rp = new RelativeParticipants(primarySensorSweep, ((StaticParticipants)allRobots), "DETECTION_UUV_NAME", RelativeParticipants.LogicOps.SUBTRACT, 1);
		
		double verifySweepRange = 30.0;
		Goal verifySensor = new Goal(entireMissionTime, rp, 
				Optional.of(new DynamicGoalRegion(primarySensorSweep, "DETECTION_COORD", verifySweepRange)),
				new SensorCover(20.0, SensorType.SONAR));
		
		mission.addGoal(mutualAvoidance);
		mission.addGoal(primarySensorSweep);
		primarySensorSweep.addSubgoal(verifySensor);
		
		// Add more message definitions here
		mission.addMessage(new Message("detectionGilda", shoreside, mission.getRobot("gilda")));
		mission.addMessage(new Message("detectionHenry", shoreside, mission.getRobot("henry")));
		mission.addMessage(new Message("detectionFrank", shoreside, mission.getRobot("frank")));
		mission.addMessage(new Message("detectionElla",  shoreside, mission.getRobot("ella")));
		return mission;
	}
}
