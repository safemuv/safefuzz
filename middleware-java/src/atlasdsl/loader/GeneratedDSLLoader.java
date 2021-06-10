package atlasdsl.loader;

import atlasdsl.*;
import atlasdsl.faults.*;
import atlassharedclasses.*;
import java.util.Optional;
import java.util.List;
import java.util.ArrayList;

import fuzzexperiment.runner.metrics.*;


public class GeneratedDSLLoader implements DSLLoader {
	public Mission loadMission() throws DSLLoadFailed {
	final double MISSION_END_TIME = 430.0;
	final boolean STOP_ON_NO_ENERGY = false;
	Mission mission = new Mission(MISSION_END_TIME, STOP_ON_NO_ENERGY);
	
	Computer c1 = new Computer("shoreside");
	mission.addComputer(c1);
	
		Robot ruav_1 = new Robot("uav_1");
		ruav_1.setPointComponentProperty("startLocation", new Point(8.0,-8.0,0.0));
		
			
		mission.addRobot(ruav_1);
		Robot ruav_2 = new Robot("uav_2");
		ruav_2.setPointComponentProperty("startLocation", new Point(8.0,8.0,0.0));
		
			
		mission.addRobot(ruav_2);
	
	
	
	
 
 
		
		Robot [] grp1 = {ruav_1,ruav_2}; 
		GoalParticipants gptrackDistances = new StaticParticipants(grp1, mission);
		
		
		
			GoalTemporalConstraints gt1 = new GoalTemporalConstraints(0.0, 2400.0);
			
			
		
		
		
		GoalAction ga1 = new TrackDistances();
		
		
		
		   
		GoalRegion grtrackDistances = new StaticGoalRegion(
			new Region(new Point(-10.0, -10.0, 0.0),
			           new Point(10.0, 10.0, 10.0)));
		
		
		Goal trackDistances = new Goal("trackDistances", mission, gt1, gptrackDistances, Optional.of(grtrackDistances), ga1);
		
		
		
		mission.addGoal("trackDistances", trackDistances);
 
 
		
		Robot [] grp2 = {ruav_1,ruav_2}; 
		GoalParticipants gpStayInOuterRegion = new StaticParticipants(grp2, mission);
		
		
		
			GoalTemporalConstraints gt2 = new GoalTemporalConstraints(0.0, MISSION_END_TIME);
		
		
		
		
		GoalAction ga2 = new StayInRegion(false);
		
		
		   
		GoalRegion grStayInOuterRegion = new StaticGoalRegion(
			new Region(new Point(-10.0, -10.0, 0.0),
			           new Point(10.0, 10.0, 10.0)));
		
		
		Goal StayInOuterRegion = new Goal("StayInOuterRegion", mission, gt2, gpStayInOuterRegion, Optional.of(grStayInOuterRegion), ga2);
		
		
		
		
		
		Metric met1 = new OutsideOfOuterRegionViolations();
		StayInOuterRegion.addMetric(met1);
		
		mission.addGoal("StayInOuterRegion", StayInOuterRegion);
 
 
		
		Robot [] grp3 = {ruav_1,ruav_2}; 
		GoalParticipants gpAvoidPlaneInner = new StaticParticipants(grp3, mission);
		
		
		
			GoalTemporalConstraints gt3 = new GoalTemporalConstraints(0.0, MISSION_END_TIME);
		
		
		
		
		GoalAction ga3 = new StayInRegion(false);
		
		
		   
		GoalRegion grAvoidPlaneInner = new StaticGoalRegion(
			new Region(new Point(-10.0, -10.0, 0.0),
			           new Point(10.0, 10.0, 10.0)));
		
		
		Goal AvoidPlaneInner = new Goal("AvoidPlaneInner", mission, gt3, gpAvoidPlaneInner, Optional.of(grAvoidPlaneInner), ga3);
		
			GoalVariable gvar1 = new GoalVariable("/lidar/distance_to_plane", "standard_msgs/Float32", true);
			AvoidPlaneInner.addVariable(gvar1);
		
		
		
		
		Metric met2 = new OutsideOfInnerRegionViolations();
		AvoidPlaneInner.addMetric(met2);
		
		mission.addGoal("AvoidPlaneInner", AvoidPlaneInner);
	

	
	
	
	return mission;
	}
}