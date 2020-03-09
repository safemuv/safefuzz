package atlasdsl;

import java.util.ArrayList;
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
	private boolean usesDynamicRegions;
	private int samplesPerUnit = DEFAULT_SAMPLES_PER_UNIT;
	private SensorType sensor;
	// Parent goal reference - set within setup
	private Goal goal;

	// This is not part of the DSL. It is used internally by the ATLAS
	// object to track the sensor state in operation
	private List<PositionTracker> posTrackers = new ArrayList<PositionTracker>();

	public CollectiveSensorCover(double density, int samplesPerUnit, SensorType sensor) {
		this.density = density;
		this.samplesPerUnit = samplesPerUnit;
		this.sensor = sensor;
	}

	protected void setup(Mission mission, Goal goal) {
		System.out.println("setup on " + goal.getName());
		Optional<GoalRegion> gr = goal.getGoalRegion();
		usesDynamicRegions = gr.isDynamic();
		
		if (gr.isPresent()) {
			Region r = gr.get().getRegion();
			this.posTrackers.add(new PositionTracker(r, samplesPerUnit));
			this.goal = goal;
		} else {
			System.out.println("no region in CollectiveSensorCover");
			throw new GoalSetupFailure(this);
			// TODO: signal a failure somehow if there is no region returned
			// throw new NoRegionForCoverageGoal();
		}
	}
	
	private void checkForNewRegions() {
		// If this is a dynamic goal region
		Optional<GoalRegion> gr = goal.getGoalRegion();
		
	}

	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		checkForNewRegions();
		super.test(mission,participants);
		try {
			// Get the coordinates of the participants operating in this goal here
			List<Robot> robots = participants.getParticipants();

			for (Robot r : robots) {
				Point coord = r.getPointComponentProperty("location");
				// Register in the position tracker some contribution from all of them
				for (PositionTracker pt : posTrackers) {
					if (pt.getRegion().contains(coord))   
						pt.notifyCoordinate(coord);
				}
			}

			// If the region is entirely scanned, then signal the completion of the goal
			boolean completed = true;
			for (PositionTracker pt : posTrackers) {
				completed = completed && pt.isComplete();
			}		
			
			if (completed) {
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
