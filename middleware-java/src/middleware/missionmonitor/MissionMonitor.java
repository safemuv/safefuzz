package middleware.missionmonitor;

import java.util.Optional;

import atlasdsl.*;
import atlasdsl.GoalResult.GoalResultStatus;
import atlassharedclasses.SonarDetection;
import middleware.core.*;

public class MissionMonitor {
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
		for (Goal g : mission.getAllGoals()) {
			g.setup();
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
						// Continue goal results have to generate a new dependent goal?
						// The primary sensor sweep detections, somehow have to be mapped into
						// the production of new dependent goals. Every time a detection is made,
						// the results productions have to develop a new subgoal which can be
						// dynamically generated
						
						// We need some concept of the result which is produced by a goal
					}
				}
			}
		}
	}
	
	public void runStep() {
		scanGoals();
	}
}
