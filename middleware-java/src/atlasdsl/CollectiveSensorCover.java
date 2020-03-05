package atlasdsl;

import java.util.Optional;

import atlassharedclasses.Region;

public class CollectiveSensorCover extends Cover {
	
	private class NoRegionForCoverageGoal extends Exception {

	}

	private double density;
	private int samplesPerUnit;
	private SensorType sensor;
	
	private final int DEFAULT_SAMPLES_PER_UNIT = 1;
	
	// This is not part of the DSL. It is used internally by the ATLAS
	// object to track the sensor state in operation
	private PositionTracker posTracker;
	
	public CollectiveSensorCover(double density, int samplesPerUnit, SensorType sensor) {
		this.density = density;
		this.samplesPerUnit = DEFAULT_SAMPLES_PER_UNIT;
		this.sensor = sensor;
		

	}
	
	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		// TODO Auto-generated method stub
		return Optional.empty();
	}

	protected void setup(Mission mission, Goal g) {
		Optional<GoalRegion> gr = g.getGoalRegion();
		if (gr.isPresent()) {
			Region r = gr.get().getRegion();
			this.posTracker = new PositionTracker(r);
		} else {
			//throw new NoRegionForCoverageGoal();
		}
	}
}
