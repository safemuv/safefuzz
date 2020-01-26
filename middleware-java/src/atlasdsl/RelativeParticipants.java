package atlasdsl;

import java.util.List;
import java.util.Optional;

public class RelativeParticipants extends GoalParticipants {
	public enum LogicOps {
		SUBTRACT,
		ADD
	}
	
	private RobotResultField robotResults;
	
	public List<Robot> getParticipants() {
		return robotResults.getRobots();
		// TODO: apply the logic operations here?
	}
}
