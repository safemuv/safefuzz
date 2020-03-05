package middleware.missionmonitor;

import java.util.Optional;

import atlasdsl.*;
import atlassharedclasses.SonarDetection;
import middleware.core.*;

public class MissionMonitor {
	private Mission mission;
	private ATLASCore core;
	
	public MissionMonitor(ATLASCore core, Mission mission) {
		this.mission = mission;	
		this.core = core;
		scanGoals();
	}
	
	public void reportDetection(SonarDetection sd) {
		// Needs to update the internal state of any sensor coverage goals 
		// which are relevant to this type of sensor
	}
		
	// TODO: needs to handle the subgoals as well here, recurse down to
	// handle them
	public void scanGoals() {
		double timeNow = core.getTime();
		for (Goal g : mission.getGoals()) {
			GoalStatus gs = g.getStatus();
			
			if ((g.getStatus() == GoalStatus.PENDING) && g.isReady(timeNow)) {
				g.setStatus(GoalStatus.STARTED);
			}
			
			if ((g.getStatus() == GoalStatus.STARTED) && g.isLate(timeNow)) {
				g.setStatus(GoalStatus.MISSED);
			} else {
				Optional<GoalResult> res = g.test();
				if (res.isPresent()) {
					System.out.println("res = " + res.get());
					g.setStatus(GoalStatus.COMPLETED);
					// This status will need to be referenced by dependent goals somehow
				}
			}
		}
	}
	
	public void runStep() {
		scanGoals();
	}
}
