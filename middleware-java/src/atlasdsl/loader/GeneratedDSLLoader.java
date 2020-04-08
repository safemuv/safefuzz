package atlasdsl.loader;

import atlasdsl.*;
import atlasdsl.faults.*;
import atlassharedclasses.*;
import java.util.Optional;

public class GeneratedDSLLoader implements DSLLoader {
	public Mission loadMission() throws DSLLoadFailed {
	
	Mission mission = new Mission();
	
	Computer c1 = new Computer("shoreside");
	mission.addComputer(c1);
	
		Robot rella = new Robot("ella");
		rella.setPointComponentProperty("startLocation", new Point(0.0,0.0,0.0));
		rella.setDoubleComponentProperty("maxSpeed", 5.0);
		rella.setDoubleComponentProperty("startSpeed", 1.0);
		
		
 
		Sensor srella_1 = new Sensor(SensorType.SONAR);
		srella_1.setParent(rella);
		srella_1.setDoubleComponentProperty("swathWidth", 10.0);
		srella_1.setDoubleComponentProperty("detectionProb", 0.99);
		rella.addSubcomponent(srella_1);
		
 
			
		mission.addRobot(rella);
		Robot rfrank = new Robot("frank");
		rfrank.setPointComponentProperty("startLocation", new Point(50.0,0.0,0.0));
		rfrank.setDoubleComponentProperty("maxSpeed", 5.0);
		rfrank.setDoubleComponentProperty("startSpeed", 1.0);
		
		
 
		Sensor srfrank_1 = new Sensor(SensorType.SONAR);
		srfrank_1.setParent(rfrank);
		srfrank_1.setDoubleComponentProperty("swathWidth", 20.0);
		srfrank_1.setDoubleComponentProperty("detectionProb", 0.99);
		rfrank.addSubcomponent(srfrank_1);
			
		mission.addRobot(rfrank);
		Robot rgilda = new Robot("gilda");
		rgilda.setPointComponentProperty("startLocation", new Point(100.0,0.0,0.0));
		rgilda.setDoubleComponentProperty("maxSpeed", 5.0);
		rgilda.setDoubleComponentProperty("startSpeed", 1.5);
		
		
 
		Sensor srgilda_1 = new Sensor(SensorType.SONAR);
		srgilda_1.setParent(rgilda);
		srgilda_1.setDoubleComponentProperty("swathWidth", 30.0);
		srgilda_1.setDoubleComponentProperty("detectionProb", 0.99);
		rgilda.addSubcomponent(srgilda_1);
			
		mission.addRobot(rgilda);
		Robot rhenry = new Robot("henry");
		rhenry.setPointComponentProperty("startLocation", new Point(150.0,0.0,0.0));
		rhenry.setDoubleComponentProperty("maxSpeed", 5.0);
		rhenry.setDoubleComponentProperty("startSpeed", 0.75);
		
		
 
		Sensor srhenry_1 = new Sensor(SensorType.SONAR);
		srhenry_1.setParent(rhenry);
		srhenry_1.setDoubleComponentProperty("swathWidth", 12.0);
		srhenry_1.setDoubleComponentProperty("detectionProb", 0.99);
		rhenry.addSubcomponent(srhenry_1);
			
		mission.addRobot(rhenry);
	
 
 
 
 
		
		Robot [] grp1 = {rella,rfrank,rgilda,rhenry}; 
		GoalParticipants gpmutualAvoidance = new StaticParticipants(grp1, mission);
		
		
		
		GoalTemporalConstraints gt1 = new GoalTemporalConstraints(0.0, 1000.0);
		
		
		GoalAction ga1 = new AvoidOthers(30.0);
		
		GoalRegion grmutualAvoidance = new StaticGoalRegion(
			new Region(new Point(0.0, 0.0, 0.0),
			           new Point(1000.0, 1000.0, 0.0)));
		
		
		Goal mutualAvoidance = new Goal("mutualAvoidance", mission, gt1, gpmutualAvoidance, Optional.of(grmutualAvoidance), ga1);
		
		
		mission.addGoal("mutualAvoidance", mutualAvoidance);
 
 
 
 
		
		Robot [] grp2 = {rella,rfrank,rgilda,rhenry}; 
		GoalParticipants gpprimarySensorSweep = new StaticParticipants(grp2, mission);
		
		
		
		GoalTemporalConstraints gt2 = new GoalTemporalConstraints(0.0, 1000.0);
		
		GoalAction ga2 = new SensorCover(10.0, 1, SensorType.SONAR);
		
		
		GoalRegion grprimarySensorSweep = new StaticGoalRegion(
			new Region(new Point(0.0, 0.0, 0.0),
			           new Point(1000.0, 1000.0, 0.0)));
		
		
		Goal primarySensorSweep = new Goal("primarySensorSweep", mission, gt2, gpprimarySensorSweep, Optional.of(grprimarySensorSweep), ga2);
		
		
		mission.addGoal("primarySensorSweep", primarySensorSweep);
		
	
		GoalParticipants gpverifySensorDetections = new RelativeParticipants(primarySensorSweep, (StaticParticipants)gpprimarySensorSweep, "UUV_NAME", RelativeParticipants.LogicOps.SUBTRACT, 1);
		
		
		GoalTemporalConstraints gt3 = new GoalTemporalConstraints(0.0, 1000.0);
		
		GoalAction ga3 = new SensorCover(5.0, 1, SensorType.SONAR);
		
		
		
		GoalRegion grverifySensorDetections = new DynamicGoalRegion(primarySensorSweep, "UUV_DETECTION_COORD", 30.0);
		
		Goal verifySensorDetections = new Goal("verifySensorDetections", mission, gt3, gpverifySensorDetections, Optional.of(grverifySensorDetections), ga3);
		
		
		mission.addGoal("verifySensorDetections", verifySensorDetections);
	
	mission.addObject(new EnvironmentalObject(1, new Point(10.0,-115.0,0.0), false));
	mission.addObject(new EnvironmentalObject(2, new Point(140.0,-65.0,0.0), true));
	mission.addObject(new EnvironmentalObject(3, new Point(135.0,-160.0,0.0), false));
	 
	Message msgDETECTION_ELLA = new Message("DETECTION_ELLA", rella, c1);
	mission.addMessage(msgDETECTION_ELLA); 
 
	Message msgDETECTION_FRANK = new Message("DETECTION_FRANK", rfrank, c1);
	mission.addMessage(msgDETECTION_FRANK); 
 
	Message msgDETECTION_GILDA = new Message("DETECTION_GILDA", rgilda, c1);
	mission.addMessage(msgDETECTION_GILDA); 
 
	Message msgDETECTION_HENRY = new Message("DETECTION_HENRY", rhenry, c1);
	mission.addMessage(msgDETECTION_HENRY); 
	
	
	
	FaultImpact fi1 = new MutateMessage(msgDETECTION_ELLA);
	
	
	
	
	FaultTimeProperties ft1 = new FaultTimeProperties(10.0, 100.0, 0.0, 0); 
	
	Fault f1 = new Fault(fi1, Optional.empty(), ft1);
	mission.addFault(f1);
	
	return mission;
	}
}