package atlasdsl.loader;

import atlasdsl.*;
import atlasdsl.faults.*;
import atlassharedclasses.*;
import java.util.Optional;
import java.util.List;
import java.util.ArrayList;

public class GeneratedDSLLoader implements DSLLoader {
	public Mission loadMission() throws DSLLoadFailed {
	final double MISSION_END_TIME = 1200.0;
	Mission mission = new Mission(MISSION_END_TIME);
	
	Computer c1 = new Computer("shoreside");
	mission.addComputer(c1);
	
		Robot rella = new Robot("ella");
		rella.setPointComponentProperty("startLocation", new Point(200.0,-85.0,0.0));
		rella.setDoubleComponentProperty("maxSpeed", 5.0);
		rella.setDoubleComponentProperty("startSpeed", 1.6);
		rella.setDoubleComponentProperty("maxDepth", 0.0);
		rella.setDoubleComponentProperty("costPerSecond", 10.0);
		
 
		Sensor srella_1 = new Sensor(SensorType.SONAR);
		srella_1.setParent(rella);
		srella_1.setDoubleComponentProperty("swathWidth", 15.0);
		srella_1.setDoubleComponentProperty("detectionProb", 0.99);
		srella_1.setDoubleComponentProperty("sensorCost", 1.0);
		rella.addSubcomponent(srella_1);
			
			
 
			
			MotionSource srella_2 = new MotionSource(0.0);
			rella.addSubcomponent(srella_2);
			
 
		Sensor srella_3 = new Sensor(SensorType.GPS_POSITION);
		srella_3.setParent(rella);
		rella.addSubcomponent(srella_3);
			
			
			
		mission.addRobot(rella);
		Robot rfrank = new Robot("frank");
		rfrank.setPointComponentProperty("startLocation", new Point(-85.0,-150.0,0.0));
		rfrank.setDoubleComponentProperty("maxSpeed", 5.0);
		rfrank.setDoubleComponentProperty("startSpeed", 1.5);
		rfrank.setDoubleComponentProperty("maxDepth", 0.0);
		
 
		Sensor srfrank_1 = new Sensor(SensorType.SONAR);
		srfrank_1.setParent(rfrank);
		srfrank_1.setDoubleComponentProperty("swathWidth", 20.0);
		srfrank_1.setDoubleComponentProperty("detectionProb", 0.99);
		rfrank.addSubcomponent(srfrank_1);
			
			
 
		Sensor srfrank_2 = new Sensor(SensorType.GPS_POSITION);
		srfrank_2.setParent(rfrank);
		rfrank.addSubcomponent(srfrank_2);
			
			
 
			
			MotionSource srfrank_3 = new MotionSource(0.0);
			rfrank.addSubcomponent(srfrank_3);
			
			
		mission.addRobot(rfrank);
		Robot rgilda = new Robot("gilda");
		rgilda.setPointComponentProperty("startLocation", new Point(190.0,-150.0,0.0));
		rgilda.setDoubleComponentProperty("maxSpeed", 5.0);
		rgilda.setDoubleComponentProperty("startSpeed", 1.5);
		rgilda.setDoubleComponentProperty("maxDepth", 0.0);
		
 
		Sensor srgilda_1 = new Sensor(SensorType.SONAR);
		srgilda_1.setParent(rgilda);
		srgilda_1.setDoubleComponentProperty("swathWidth", 10.0);
		srgilda_1.setDoubleComponentProperty("detectionProb", 0.99);
		rgilda.addSubcomponent(srgilda_1);
			
			
 
		Sensor srgilda_2 = new Sensor(SensorType.GPS_POSITION);
		srgilda_2.setParent(rgilda);
		rgilda.addSubcomponent(srgilda_2);
			
			
 
			
			MotionSource srgilda_3 = new MotionSource(0.0);
			rgilda.addSubcomponent(srgilda_3);
			
			
		mission.addRobot(rgilda);
		Robot rhenry = new Robot("henry");
		rhenry.setPointComponentProperty("startLocation", new Point(-85.0,-45.0,0.0));
		rhenry.setDoubleComponentProperty("maxSpeed", 5.0);
		rhenry.setDoubleComponentProperty("startSpeed", 1.5);
		rhenry.setDoubleComponentProperty("maxDepth", 0.0);
		
 
		Sensor srhenry_1 = new Sensor(SensorType.SONAR);
		srhenry_1.setParent(rhenry);
		srhenry_1.setDoubleComponentProperty("swathWidth", 25.0);
		srhenry_1.setDoubleComponentProperty("detectionProb", 0.99);
		rhenry.addSubcomponent(srhenry_1);
			
			
 
		Sensor srhenry_2 = new Sensor(SensorType.GPS_POSITION);
		srhenry_2.setParent(rhenry);
		rhenry.addSubcomponent(srhenry_2);
			
			
 
			
			MotionSource srhenry_3 = new MotionSource(0.0);
			rhenry.addSubcomponent(srhenry_3);
			
			
		mission.addRobot(rhenry);
	
	
	EnvironmentalObject eo1 = new EnvironmentalObject(1, new Point(0.0,-55.0,0.0), false);
	mission.addObject(eo1);
	EnvironmentalObject eo2 = new EnvironmentalObject(2, new Point(185.0,-45.0,0.0), true);
	mission.addObject(eo2);
	EnvironmentalObject eo3 = new EnvironmentalObject(3, new Point(100.0,-132.0,0.0), false);
	mission.addObject(eo3);
	
	
 
 
 
 
		
		Robot [] grp1 = {rella,rfrank,rgilda,rhenry}; 
		GoalParticipants gpmutualAvoidance = new StaticParticipants(grp1, mission);
		
		
		
		GoalTemporalConstraints gt1 = new GoalTemporalConstraints(0.0, 1200.0);
		
		
		GoalAction ga1 = new AvoidOthers(4.0);
		
		
		
		
		GoalRegion grmutualAvoidance = new StaticGoalRegion(
			new Region(new Point(0.0, 0.0, 0.0),
			           new Point(1000.0, 1000.0, 0.0)));
		
		
		Goal mutualAvoidance = new Goal("mutualAvoidance", mission, gt1, gpmutualAvoidance, Optional.of(grmutualAvoidance), ga1);
		
		
		mission.addGoal("mutualAvoidance", mutualAvoidance);
 
 
 
 
		
		Robot [] grp2 = {rella,rfrank,rgilda,rhenry}; 
		GoalParticipants gpprimarySensorSweep = new StaticParticipants(grp2, mission);
		
		
		
		GoalTemporalConstraints gt2 = new GoalTemporalConstraints(0.0, 1200.0);
		
		GoalAction ga2 = new SensorCover(10.0, 1, SensorType.SONAR, 1, 2);
		
		
		
		
		
		GoalRegion grprimarySensorSweep = new StaticGoalRegion(
			new Region(new Point(-50.0, -230.0, 0.0),
			           new Point(200.0, -30.0, 0.0)));
		
		
		Goal primarySensorSweep = new Goal("primarySensorSweep", mission, gt2, gpprimarySensorSweep, Optional.of(grprimarySensorSweep), ga2);
		
		
		mission.addGoal("primarySensorSweep", primarySensorSweep);
		
	
		GoalParticipants gpverifySensorDetections = new RelativeParticipants(primarySensorSweep, (StaticParticipants)gpprimarySensorSweep, "UUV_NAME", RelativeParticipants.LogicOps.SUBTRACT, 1);
		
		
		GoalTemporalConstraints gt3 = new GoalTemporalConstraints(0.0, 1200.0);
		
		GoalAction ga3 = new SensorCover(5.0, 1, SensorType.SONAR, 1, 2);
		
		
		
		
		
		
		GoalRegion grverifySensorDetections = new DynamicGoalRegion(primarySensorSweep, "UUV_DETECTION_COORD", 30.0);
		
		Goal verifySensorDetections = new Goal("verifySensorDetections", mission, gt3, gpverifySensorDetections, Optional.of(grverifySensorDetections), ga3);
		
		try {
			verifySensorDetections.setDependencyOn(primarySensorSweep);
		} catch (SelfDependencyError e) {
			throw new DSLLoadFailed("Goal verifySensorDetections depends on itself");
		}
		
		mission.addGoal("verifySensorDetections", verifySensorDetections);
 
 
 
 
		
		Robot [] grp4 = {rella,rfrank,rgilda,rhenry}; 
		GoalParticipants gpfindTestObjects = new StaticParticipants(grp4, mission);
		
		
		
		GoalTemporalConstraints gt4 = new GoalTemporalConstraints(0.0, 1200.0);
		
		
		
		
		
		List<EnvironmentalObject> ga4Objs = new ArrayList<EnvironmentalObject>();
		ga4Objs.add(eo1);
		ga4Objs.add(eo3);
		ga4Objs.add(eo2);
		GoalAction ga4 = new DiscoverObjects(ga4Objs, 2);
		
		
		GoalRegion grfindTestObjects = new StaticGoalRegion(
			new Region(new Point(0.0, 0.0, 0.0),
			           new Point(0.0, 0.0, 0.0)));
		
		
		Goal findTestObjects = new Goal("findTestObjects", mission, gt4, gpfindTestObjects, Optional.of(grfindTestObjects), ga4);
		
		
		mission.addGoal("findTestObjects", findTestObjects);
 
 
 
 
		
		Robot [] grp5 = {rella,rfrank,rgilda,rhenry}; 
		GoalParticipants gpstayInRegion = new StaticParticipants(grp5, mission);
		
		
		
		GoalTemporalConstraints gt5 = new GoalTemporalConstraints(0.0, 1200.0);
		
		
		
		
		GoalAction ga5 = new StayInRegion(false);
		
		
		GoalRegion grstayInRegion = new StaticGoalRegion(
			new Region(new Point(-150.0, -260.0, -40.0),
			           new Point(245.0, 20.0, 100.0)));
		
		
		Goal stayInRegion = new Goal("stayInRegion", mission, gt5, gpstayInRegion, Optional.of(grstayInRegion), ga5);
		
		
		mission.addGoal("stayInRegion", stayInRegion);
 
 
 
 
		
		Robot [] grp6 = {rella,rfrank,rgilda,rhenry}; 
		GoalParticipants gptrackDistances = new StaticParticipants(grp6, mission);
		
		
		
		GoalTemporalConstraints gt6 = new GoalTemporalConstraints(0.0, 1190.0);
		
		
		
		GoalAction ga6 = new TrackDistances();
		
		
		
		GoalRegion grtrackDistances = new StaticGoalRegion(
			new Region(new Point(-150.0, -260.0, -40.0),
			           new Point(245.0, 20.0, 100.0)));
		
		
		Goal trackDistances = new Goal("trackDistances", mission, gt6, gptrackDistances, Optional.of(grtrackDistances), ga6);
		
		
		mission.addGoal("trackDistances", trackDistances);
	

	
	
	
	return mission;
	}
}