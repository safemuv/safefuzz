package atlasdsl;

import java.util.ArrayList;
import java.util.List;

public class RelativeParticipants extends GoalParticipants {
	public enum LogicOps {
		SUBTRACT,
		ADD
	}
	
	private Goal parentGoal;
	private String fieldName;
	private int participantLimit;
	private StaticParticipants staticRobots;
	private LogicOps operation;
	
	public RelativeParticipants(Goal parentGoal, StaticParticipants staticRobots, String fieldName, LogicOps operation, int participantLimit) {
		this.parentGoal = parentGoal;
		this.fieldName = fieldName;
		this.staticRobots = staticRobots;
		this.operation = operation;
		this.participantLimit = participantLimit;
	}
	
	public List<Robot> getParticipants() {
		return new ArrayList<Robot>();
		// TODO: compute this dynamically
	}
}
