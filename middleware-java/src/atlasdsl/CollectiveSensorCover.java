package atlasdsl;

import java.util.Optional;

public class CollectiveSensorCover extends Cover {
	private double density;
	private int samplesPerUnit;
	private SensorType sensor;
	
	private final int DEFAULT_SAMPLES_PER_UNIT = 1;
	
	// This is not part of the DSL. It is used internally by the ATLAS
	// object to track the sensor state in operation
	private PositionTracker postracker = new PositionTracker();
	
	public CollectiveSensorCover(double density, SensorType sensor) {
		this.density = density;
		this.samplesPerUnit = DEFAULT_SAMPLES_PER_UNIT;
		this.sensor = sensor;
	}
	
	public CollectiveSensorCover(double density, int samplesPerUnit, SensorType sensor) {
		this.density = density;
		this.samplesPerUnit = samplesPerUnit;
		this.sensor = sensor;
	}
	
	@Override
	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		// TODO Auto-generated method stub
		return Optional.empty();
		
	}

}
