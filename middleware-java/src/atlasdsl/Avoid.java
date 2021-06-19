package atlasdsl;

import java.util.Optional;

import middleware.core.ATLASCore;

public class Avoid extends GoalAction {
	private double clearance;
	
	public Avoid(double d) {
		this.clearance = clearance;
	}

	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		// TODO Auto-generated method stub
		return Optional.empty();
	}

	protected void setup(ATLASCore core, Mission mission, Goal g) {
		// TODO Auto-generated method stub
	}
}
