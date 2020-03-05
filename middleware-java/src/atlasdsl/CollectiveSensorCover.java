package atlasdsl;

import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import atlasdsl.GoalResult.GoalResultStatus;
import atlassharedclasses.Point;
import atlassharedclasses.Region;

public class CollectiveSensorCover extends Cover {
	private class NoRegionForCoverageGoal extends Exception {

	}

	private final int DEFAULT_SAMPLES_PER_UNIT = 1;

	private double density;
	private int samplesPerUnit = DEFAULT_SAMPLES_PER_UNIT;
	private SensorType sensor;

	// This is not part of the DSL. It is used internally by the ATLAS
	// object to track the sensor state in operation
	private PositionTracker posTracker;

	public CollectiveSensorCover(double density, int samplesPerUnit, SensorType sensor) {
		this.density = density;
		this.samplesPerUnit = samplesPerUnit;
		this.sensor = sensor;
	}

	protected void setup(Mission mission, Goal g) {
		System.out.println("setup on " + g.getName());
		Optional<GoalRegion> gr = g.getGoalRegion();
		if (gr.isPresent()) {
			Region r = gr.get().getRegion();
			this.posTracker = new PositionTracker(r, samplesPerUnit);
		} else {
			System.out.println("no region in CollectiveSensorCover");
			// TODO: signal a failure somehow if there is no region returned
			// throw new NoRegionForCoverageGoal();
		}
	}

	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		try {
			// Get the coordinates of the participants operating in this goal here
			List<Robot> robots = participants.getParticipants();

			for (Robot r : robots) {
				Point coord = r.getPointComponentProperty("location");
				// Register in the position tracker some contribution from all of them
				posTracker.notifyCoordinate(coord);
			}

			// If the region is entirely scanned, then signal the completion of the goal
			if (posTracker.isComplete()) {
				GoalResult gr = new GoalResult(GoalResultStatus.COMPLETED);
				return Optional.of(gr);
			} else {
				return Optional.empty();
			}

		} catch (MissingProperty p) {
			return Optional.empty();
		}
	}
}
