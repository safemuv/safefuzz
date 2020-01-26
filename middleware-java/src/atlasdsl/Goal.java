package atlasdsl;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Goal {
	private GoalTemporalConstraints timingReqs;
	private GoalParticipants participants;
	private List<Goal> subgoals = new ArrayList<Goal>();
	private GoalAction action;
	
	private GoalStatus status = GoalStatus.PENDING;
	
	public boolean isReady(double timeNow) {
		return timingReqs.isReady(timeNow);
	}
	
	public GoalStatus getStatus() {
		return status;
	}
	
	public void setStatus(GoalStatus gs) {
		status = gs;
	} 

	public boolean isLate(double timeNow) {
		return timingReqs.isLate(timeNow);
	}
	
	public Optional<GoalResult> test() {
		return action.test(this.participants);
	}
}
