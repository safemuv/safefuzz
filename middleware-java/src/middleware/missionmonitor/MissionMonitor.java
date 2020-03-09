package middleware.missionmonitor;

import java.util.Optional;

import atlasdsl.*;
import atlasdsl.GoalResult.GoalResultStatus;
import atlassharedclasses.SonarDetection;
import middleware.core.*;

public class MissionMonitor {
	private class MissionMonitorSetupFailure extends Exception {
		
	}
	
	private Mission mission;
	private ATLASCore core;
	
	public MissionMonitor(ATLASCore core, Mission mission) {
		this.mission = mission;	
		this.core = core;
		setupGoals();
		scanGoals();
	}
	
	public void reportDetection(SonarDetection sd) {
		// Needs to update the internal state of any sensor coverage goals 
		// which are relevant to this type of sensor
	}
	
	public void setupGoals() {
		try {
			for (Goal g : mission.getAllGoals()) {
				g.setup();
			}
		} catch (GoalActionSetupFailure gs) {
			System.out.println("Goal action setup failure");
		}
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
					GoalResult gr = res.get();
					
					// TODO: This GoalResult needs to be stored in a database, so its fields can be 
					// referenced by dependent goals somehow
					
					System.out.println("res = " + res.get());
					if (gr.getResultStatus() == GoalResultStatus.VIOLATED) {
						g.setStatus(GoalStatus.VIOLATED);
					}
					
					if (gr.getResultStatus() == GoalResultStatus.COMPLETED) {
						g.setStatus(GoalStatus.COMPLETED);
					}
					
					if (gr.getResultStatus() == GoalResultStatus.CONTINUE) {

					}
				}
			}
		}
	}
	
	public void runStep() {
		scanGoals();
	}
}
