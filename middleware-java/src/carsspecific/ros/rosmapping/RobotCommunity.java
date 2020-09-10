package carsspecific.ros.rosmapping;

import atlasdsl.*;
import atlassharedclasses.Point;

public class RobotCommunity extends MOOSCommunity {
	// TODO: Add custom behaviours in constructor - must include a Waypoint for each
	// robot to be updated by the collective intelligence
	public RobotCommunity(MOOSSimulation sim, Robot robot, Point startPos, double startSpeed, double maxSpeed, double maxDepth, int pSharePortBase) {
		super(sim,robot.getName());	
	}
	
	public void addSonarSensors() {
		
	}
}
