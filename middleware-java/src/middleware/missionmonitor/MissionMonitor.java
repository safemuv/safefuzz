package middleware.missionmonitor;

import java.util.Optional;

import atlasdsl.*;
import atlassharedclasses.SonarDetection;
import middleware.core.*;

public class MissionMonitor {
	private Mission mission;
	double timeNow;
	
	public MissionMonitor(Mission mission, double startingMissionTime) {
		this.mission = mission;
		this.timeNow = startingMissionTime;
		scanGoals();
	}
	
	public void reportDetection(SonarDetection sd) {
		// Needs to update the internal state of any sensor coverage goals 
		// which are relevant to this type of sensor
	}
	
	// TODO: needs to handle the subgoals as well here, recurse down to
	// handle them
	public void scanGoals() {
		for (Goal g : mission.getGoals()) {
			if ((g.getStatus() == GoalStatus.PENDING) && g.isReady(timeNow)) {
				g.setStatus(GoalStatus.STARTED);
			}
			
			if ((g.getStatus() == GoalStatus.STARTED) && g.isLate(timeNow)) {
				g.setStatus(GoalStatus.MISSED);
			} else {
				Optional<GoalResult> res = g.test();
				if (res.isPresent()) {
					g.setStatus(GoalStatus.COMPLETED);
					// This status will need to be referenced by dependent goals somehow
				}
			}
		}
	}
	
	public void advanceTime(double newTime) throws CausalityException {
		if (newTime > timeNow) {
			timeNow = newTime;
		} else {
			throw new CausalityException(newTime, timeNow);
		}
	}
	
	// TODO: need to get the time from MOOS in some way!
	// Can ATLASDBWatch enclose it in all variable updates?
	public void runStep(double newTime) throws CausalityException {
		advanceTime(newTime);
		scanGoals();
	}
}
