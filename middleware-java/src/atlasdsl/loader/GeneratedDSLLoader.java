package atlasdsl.loader;

import atlasdsl.*;
import atlassharedclasses.*;
import java.util.Optional;

public class GeneratedDSLLoader implements DSLLoader {
	public Mission loadMission() throws DSLLoadFailed {
		
	Mission mission = new Mission();
	
	Computer c0 = new Computer("shoreside");
	mission.addComputer(c0);
	
		Robot rella = new Robot("ella");
		rella.setPointComponentProperty("startLocation", new Point(30.0,20.0,0.0));
		rella.setDoubleComponentProperty("startSpeed", 1.0);
		rella.setDoubleComponentProperty("maxSpeed", 5.0);
		
 
		Sensor srella_1 = new Sensor(SensorType.SONAR);
		srella_1.setDoubleComponentProperty("swathWidth", 25.0);
		srella_1.setDoubleComponentProperty("detectionProb", 0.99);
		rella.addSubcomponent(srella_1);
			
		mission.addRobot(rella);
		Robot rfrank = new Robot("frank");
		rfrank.setPointComponentProperty("startLocation", new Point(60.0,0.0,0.0));
		rfrank.setDoubleComponentProperty("startSpeed", 2.0);
		rfrank.setDoubleComponentProperty("maxSpeed", 5.0);
		
 
		Sensor srfrank_1 = new Sensor(SensorType.SONAR);
		srfrank_1.setDoubleComponentProperty("swathWidth", 50.0);
		srfrank_1.setDoubleComponentProperty("detectionProb", 0.98);
		rfrank.addSubcomponent(srfrank_1);
			
		mission.addRobot(rfrank);
		Robot rhenry = new Robot("henry");
		rhenry.setPointComponentProperty("startLocation", new Point(90.0,0.0,0.0));
		rhenry.setDoubleComponentProperty("startSpeed", 0.75);
		rhenry.setDoubleComponentProperty("maxSpeed", 5.0);
		
 
		Sensor srhenry_1 = new Sensor(SensorType.SONAR);
		srhenry_1.setDoubleComponentProperty("swathWidth", 50.0);
		srhenry_1.setDoubleComponentProperty("detectionProb", 0.98);
		rhenry.addSubcomponent(srhenry_1);
			
		mission.addRobot(rhenry);
		Robot rgilda = new Robot("gilda");
		rgilda.setPointComponentProperty("startLocation", new Point(120.0,0.0,0.0));
		rgilda.setDoubleComponentProperty("startSpeed", 1.6);
		rgilda.setDoubleComponentProperty("maxSpeed", 5.0);
		
 
		Sensor srgilda_1 = new Sensor(SensorType.SONAR);
		srgilda_1.setDoubleComponentProperty("swathWidth", 12.0);
		srgilda_1.setDoubleComponentProperty("detectionProb", 0.98);
		rgilda.addSubcomponent(srgilda_1);
			
		mission.addRobot(rgilda);
	
 
 
 
 
		
		Robot [] grp1 = {rella,rfrank,rgilda,rhenry}; 
		GoalParticipants gpmutualAvoidance = new StaticParticipants(grp1, mission);
		
		
		
		GoalTemporalConstraints gt1 = new GoalTemporalConstraints(0.0, 10000.0);
		
		
		GoalAction ga1 = new AvoidOthers(3.0);
		
		Goal mutualAvoidance = new Goal("mutualAvoidance", mission, gt1, gpmutualAvoidance, Optional.empty(), ga1);
		
		
		mission.addGoal("mutualAvoidance", mutualAvoidance);
 
 
 
 
		
		Robot [] grp2 = {rella,rfrank,rgilda,rhenry}; 
		GoalParticipants gpprimarySensorSweep = new StaticParticipants(grp2, mission);
		
		
		
		GoalTemporalConstraints gt2 = new GoalTemporalConstraints(0.0, 10000.0);
		
		GoalAction ga2 = new SensorCover(10.0, 1, SensorType.SONAR);
		
		
		Goal primarySensorSweep = new Goal("primarySensorSweep", mission, gt2, gpprimarySensorSweep, Optional.empty(), ga2);
		
		
		mission.addGoal("primarySensorSweep", primarySensorSweep);
		
	
		GoalParticipants gpverifyDetections = new RelativeParticipants(primarySensorSweep, (StaticParticipants)gpprimarySensorSweep, "UUV_DETECTION_NAME", RelativeParticipants.LogicOps.ADD, 1);
		
		
		GoalTemporalConstraints gt3 = new GoalTemporalConstraints(0.0, 10000.0);
		
		GoalAction ga3 = new SensorCover(10.0, 3, SensorType.SONAR);
		
		
		Goal verifyDetections = new Goal("verifyDetections", mission, gt3, gpverifyDetections, Optional.empty(), ga3);
		
		try {
			verifyDetections.setDependencyOn(primarySensorSweep);
		} catch (SelfDependencyError e) {
			throw new DSLLoadFailed();
		}
		
		mission.addGoal("verifyDetections", verifyDetections);
	
	mission.addObject(new EnvironmentalObject(1, new Point(10.0,-115.0,0.0), false));
	mission.addObject(new EnvironmentalObject(2, new Point(140.0,-65.0,0.0), true));
	mission.addObject(new EnvironmentalObject(0, new Point(135.0,-160.0,0.0), false));
	mission.addObject(new EnvironmentalObject(4, new Point(50.0,50.0,0.0), false));
	
	return mission;
	}
}