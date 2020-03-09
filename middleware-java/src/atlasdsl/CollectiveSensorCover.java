package atlasdsl;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
	// Parent goal and GoalRegion reference - set within setup
	private Goal goal;
	private GoalRegion goalRegion;
	private int regionCount;

	// This is not part of the DSL. It is used internally by the ATLAS
	// object to track the sensor state in operation
	private Map<Region,PositionTracker> posTrackers = new HashMap<Region,PositionTracker>();

	public CollectiveSensorCover(double density, int samplesPerUnit, SensorType sensor) {
		this.density = density;
		this.samplesPerUnit = samplesPerUnit;
		this.sensor = sensor;
	}

	protected void setup(Mission mission, Goal goal) throws GoalActionSetupFailure {
		System.out.println("setup on " + goal.getName());
		this.goal = goal;
		Optional<GoalRegion> gr_o = goal.getGoalRegion();
		//usesDynamicRegions = gr.isDynamic();
		
		if (gr_o.isPresent()) {
			this.goalRegion = gr_o.get();
			List<Region> regions = gr_o.get().getRegions();
			regionCount = regions.size();
			
			for (Region r : regions) {
				posTrackers.put(r, new PositionTracker(r, samplesPerUnit));
			}
				
			
		} else { 
			System.out.println("no region in CollectiveSensorCover");
			throw new GoalActionSetupFailure(this);
			// TODO: signal a failure somehow if there is no region returned
			// throw new NoRegionForCoverageGoal();
		}
	}
	
	private void checkForNewRegions() {
		// Put any new regions into the position trackers map
		if (goalRegion.getRegionCount() > regionCount) {
			List<Region> regions = goalRegion.getRegions();
			for (Region r : regions) {
				if (!posTrackers.containsKey(r)) { 
					posTrackers.put(r, new PositionTracker(r, samplesPerUnit));
				}
			}
		}
		
	}

	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		
		if (goalRegion.isDynamic()) {
			checkForNewRegions();
		}
		
		super.test(mission,participants);
		try {
			// Get the coordinates of the participants operating in this goal here
			List<Robot> robots = participants.getParticipants();

			for (Robot r : robots) {
				Point coord = r.getPointComponentProperty("location");
				// Register in the position tracker some contribution from all of them
				for (PositionTracker pt : posTrackers.values()) {
					if (pt.getRegion().contains(coord))   
						pt.notifyCoordinate(coord);
				}
			}

			// If the region is entirely scanned, then signal the completion of the goal
			boolean completed = true;
			for (PositionTracker pt : posTrackers.values()) {
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
