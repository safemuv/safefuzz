package atlasdsl;

import java.util.Optional;

public class SensorCover extends Cover {
	private double density;
	private SensorType sensor;

	public SensorCover(double d, SensorType sensor) {
		this.density = d;
		this.sensor = sensor;
	}

	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		// TODO Auto-generated method stub
		return Optional.empty();
	}

	protected void setup(Mission mission, Goal g) {
		// TODO Auto-generated method stub		
	}
}
