package atlasdsl;

import java.util.List;
import java.util.Optional;

import atlasdsl.GoalResult.GoalResultStatus;
import atlassharedclasses.Point;
import atlassharedclasses.Region;
import middleware.core.ATLASCore;

// Assume StayInRegion requires a staticGoalRegion currently
public class StayInRegion extends GoalAction {
	private Region region;
	
	public class MissingRegion extends Exception {
		private static final long serialVersionUID = 1L;
		
	}

	public StayInRegion() {
		// The region is determined during the setup method
	}
	
	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		List<Robot> robots = participants.getParticipants();
		try {
			for (Robot r : robots) {
				Point loc = r.getPointComponentProperty("location");
				if (!region.contains(loc)) {
					GoalResult gr = new GoalResult(GoalResultStatus.VIOLATED);
					GoalResultField gf = new StringResultField("violatingRobot", r.getName());
					GoalResultField pf = new PointResultField("location", loc);
					gr.addField(gf);
					gr.addField(pf);
					return Optional.of(gr);
				}
			}
		} catch (MissingProperty mp) {
			// TODO: some properties missing for robots - declare that superclass throws GoalTestFailure
			// or similar in this case.
			return Optional.empty();
		}
		return Optional.empty();
	}

	protected void setup(ATLASCore core, Mission mission, Goal g) throws GoalActionSetupFailure {
		Optional<GoalRegion> gr_o = g.getGoalRegion();
		if (gr_o.isPresent()) {
			GoalRegion gr = gr_o.get();
			if (gr instanceof StaticGoalRegion) {
				Region r = ((StaticGoalRegion) gr).getRegion();
				this.region = r;
			} else {
				throw new GoalActionSetupFailure(this, "StayInRegion action requires a StaticGoalRegion associated with it");
			}
		} else {
			throw new GoalActionSetupFailure(this, "No associated region for a StayInRegion object");
		}
	}
}
