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
	
		Robot rgilda = new Robot("gilda");
		rgilda.setPointComponentProperty("startLocation", new Point(15.0,0.0,0.0));
		rgilda.setDoubleComponentProperty("maxSpeed", 5.0);
		rgilda.setDoubleComponentProperty("startSpeed", 1.5);
		rgilda.setDoubleComponentProperty("maxDepth", 0.0);
		
 
		Sensor srgilda_1 = new Sensor(SensorType.SONAR);
		srgilda_1.setParent(rgilda);
		srgilda_1.setDoubleComponentProperty("swathWidth", 10.0);
		srgilda_1.setDoubleComponentProperty("detectionProb", 0.99);
		rgilda.addSubcomponent(srgilda_1);
			
			
			
			
			
		mission.addRobot(rgilda);
	
	
	
	ArrayList<Point> eopoints0 = new ArrayList<Point>();
		eopoints0.add(new Point(107.0, -101.0, 0.0));
		eopoints0.add(new Point(112.0, -106.0, 0.0));
		eopoints0.add(new Point(112.0, -113.0, 0.0));
		eopoints0.add(new Point(107.0, -118.0, 0.0));
		eopoints0.add(new Point(100.0, -118.0, 0.0));
		eopoints0.add(new Point(95.0, -113.0, 0.0));
		eopoints0.add(new Point(95.0, -106.0, 0.0));
		eopoints0.add(new Point(100.0, -101.0, 0.0));
	EnvironmentalObstacle eob0 = new EnvironmentalObstacle("ob_0", eopoints0);
	mission.addObstacle(eob0);
	ArrayList<Point> eopoints1 = new ArrayList<Point>();
		eopoints1.add(new Point(50.0, -30.0, 0.0));
		eopoints1.add(new Point(53.0, -33.0, 0.0));
		eopoints1.add(new Point(53.0, -38.0, 0.0));
		eopoints1.add(new Point(50.0, -42.0, 0.0));
		eopoints1.add(new Point(45.0, -42.0, 0.0));
		eopoints1.add(new Point(41.0, -38.0, 0.0));
		eopoints1.add(new Point(41.0, -33.0, 0.0));
		eopoints1.add(new Point(45.0, -30.0, 0.0));
	EnvironmentalObstacle eob1 = new EnvironmentalObstacle("ob_1", eopoints1);
	mission.addObstacle(eob1);
	ArrayList<Point> eopoints2 = new ArrayList<Point>();
		eopoints2.add(new Point(82.0, -65.0, 0.0));
		eopoints2.add(new Point(87.0, -70.0, 0.0));
		eopoints2.add(new Point(87.0, -76.0, 0.0));
		eopoints2.add(new Point(82.0, -81.0, 0.0));
		eopoints2.add(new Point(76.0, -81.0, 0.0));
		eopoints2.add(new Point(71.0, -76.0, 0.0));
		eopoints2.add(new Point(71.0, -70.0, 0.0));
		eopoints2.add(new Point(76.0, -65.0, 0.0));
	EnvironmentalObstacle eob2 = new EnvironmentalObstacle("ob_2", eopoints2);
	mission.addObstacle(eob2);
	ArrayList<Point> eopoints3 = new ArrayList<Point>();
		eopoints3.add(new Point(59.0, -101.0, 0.0));
		eopoints3.add(new Point(64.0, -105.0, 0.0));
		eopoints3.add(new Point(64.0, -112.0, 0.0));
		eopoints3.add(new Point(59.0, -116.0, 0.0));
		eopoints3.add(new Point(53.0, -116.0, 0.0));
		eopoints3.add(new Point(48.0, -112.0, 0.0));
		eopoints3.add(new Point(48.0, -105.0, 0.0));
		eopoints3.add(new Point(53.0, -101.0, 0.0));
	EnvironmentalObstacle eob3 = new EnvironmentalObstacle("ob_3", eopoints3);
	mission.addObstacle(eob3);
	ArrayList<Point> eopoints4 = new ArrayList<Point>();
		eopoints4.add(new Point(107.0, -38.0, 0.0));
		eopoints4.add(new Point(112.0, -43.0, 0.0));
		eopoints4.add(new Point(112.0, -49.0, 0.0));
		eopoints4.add(new Point(107.0, -54.0, 0.0));
		eopoints4.add(new Point(100.0, -54.0, 0.0));
		eopoints4.add(new Point(96.0, -49.0, 0.0));
		eopoints4.add(new Point(96.0, -43.0, 0.0));
		eopoints4.add(new Point(100.0, -38.0, 0.0));
	EnvironmentalObstacle eob4 = new EnvironmentalObstacle("ob_4", eopoints4);
	mission.addObstacle(eob4);
	
	Region eregion = new Region(new Point(30.0, -140.0, 0.0),
								new Point(120.0, -20.0, 0.0));
	mission.setObstacleRegion(eregion);
	
	
 
		
		Robot [] grp1 = {rgilda}; 
		GoalParticipants gpmutualAvoidance = new StaticParticipants(grp1, mission);
		
		
		
		GoalTemporalConstraints gt1 = new GoalTemporalConstraints(0.0, 1200.0);
		
		
		GoalAction ga1 = new AvoidOthers(4.0);
		
		
		
		
		GoalRegion grmutualAvoidance = new StaticGoalRegion(
			new Region(new Point(0.0, 0.0, 0.0),
			           new Point(1000.0, 1000.0, 0.0)));
		
		
		Goal mutualAvoidance = new Goal("mutualAvoidance", mission, gt1, gpmutualAvoidance, Optional.of(grmutualAvoidance), ga1);
		
		
		mission.addGoal("mutualAvoidance", mutualAvoidance);
 
		
		Robot [] grp2 = {rgilda}; 
		GoalParticipants gptrackDistances = new StaticParticipants(grp2, mission);
		
		
		
		GoalTemporalConstraints gt2 = new GoalTemporalConstraints(0.0, 1201.0);
		
		
		
		GoalAction ga2 = new TrackDistances();
		
		
		
		GoalRegion grtrackDistances = new StaticGoalRegion(
			new Region(new Point(0.0, 0.0, 0.0),
			           new Point(0.0, 0.0, 0.0)));
		
		
		Goal trackDistances = new Goal("trackDistances", mission, gt2, gptrackDistances, Optional.of(grtrackDistances), ga2);
		
		
		mission.addGoal("trackDistances", trackDistances);
	

	
	
	
	return mission;
	}
}