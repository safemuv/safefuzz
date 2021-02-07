package atlasdsl;

import java.util.List;
import java.util.Optional;

import atlasdsl.GoalResult.GoalResultStatus;
import atlassharedclasses.Point;
import atlassharedclasses.Region;
import middleware.core.ATLASCore;

public class StayInRegion extends GoalAction {
	private Region region;
	
	public StayInRegion(Region region) {
		this.region = region;
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
			// TODO: some properties missing for robots
			return Optional.empty();
		}
		return Optional.empty();
	}

	protected void setup(ATLASCore core, Mission mission, Goal g) {
		
	}
}
