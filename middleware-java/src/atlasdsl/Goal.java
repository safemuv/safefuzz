package atlasdsl;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import atlassharedclasses.Region;

public class Goal {
	private String name;
	protected Mission mission;
	private GoalTemporalConstraints timingReqs;
	private GoalParticipants participants;
	private Optional<GoalRegion> region;
	private List<Goal> subgoals = new ArrayList<Goal>();
	private GoalAction action;
	
	public Goal(String name, Mission mission, GoalTemporalConstraints timingReqs, GoalParticipants participants, Optional<GoalRegion> region, GoalAction action) {
		this.name = name;
		this.mission = mission;
		this.timingReqs = timingReqs;
		this.participants = participants;
		this.region = region;
		this.action = action;
	}
	
	private GoalStatus status = GoalStatus.PENDING;
	
	public boolean isReady(double timeNow) {
		return timingReqs.isReady(timeNow);
	}
	
	public GoalStatus getStatus() {
		return status;
	}
	
	public String getName() {
		return name;
	}
	
	public void setStatus(GoalStatus gs) {
		status = gs;
	} 

	public boolean isLate(double timeNow) {
		return timingReqs.isLate(timeNow);
	}
	
	public Optional<GoalResult> test() {
		return action.test(mission, this.participants);
	}
	
	public void setup() {
		
	}

	public void addSubgoal(Goal sg) {
		subgoals.add(sg);
	}

	public Optional<GoalRegion> getGoalRegion() {
		return region;
	}
}
