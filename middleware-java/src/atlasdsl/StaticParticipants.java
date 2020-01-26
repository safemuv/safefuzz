package atlasdsl;

import java.util.List;

public class StaticParticipants extends GoalParticipants {
	List<Robot> participants;
	
	public StaticParticipants(List<Robot> participants) {
		this.participants = participants;
	}

	public List<Robot> getParticipants() {
		// TODO Auto-generated method stub
		return participants;
	}
}
