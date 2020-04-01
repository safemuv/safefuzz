
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
		
 
		Sensor srella_1 = new Sensor(SensorType.SONAR);
		rella.addSubcomponent(srella_1);
			
		mission.addRobot(rella);
		Robot rfrank = new Robot("frank");
		
 
		Sensor srfrank_1 = new Sensor(SensorType.SONAR);
		rfrank.addSubcomponent(srfrank_1);
			
		mission.addRobot(rfrank);
		Robot rgilda = new Robot("gilda");
		
 
		Sensor srgilda_1 = new Sensor(SensorType.SONAR);
		rgilda.addSubcomponent(srgilda_1);
			
		mission.addRobot(rgilda);
		Robot rhenry = new Robot("henry");
		
 
		Sensor srhenry_1 = new Sensor(SensorType.SONAR);
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
	
	
	return mission;
	}
}