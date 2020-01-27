package atlasdsl;

import java.util.Optional;

public class SensorCover extends Cover {
	private double density;
	private SensorType sensor;

	public SensorCover(double d, SensorType sensor) {
		this.density = d;
		this.sensor = sensor;
	}

	protected Optional<GoalResult> test(GoalParticipants participants) {
		// TODO Auto-generated method stub
		return Optional.empty();
	}
}
