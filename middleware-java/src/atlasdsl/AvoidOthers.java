package atlasdsl;

import java.util.List;
import java.util.Optional;

import atlasdsl.GoalResult.GoalResultStatus;
import atlassharedclasses.Point;

public class AvoidOthers extends GoalAction {
	private double clearance;
	
	public AvoidOthers(double clearance) {
		this.clearance = clearance;
	}

	// TODO: Specific log for the computation of each goal?
	private Optional<Double> testDistances(List<Robot> robots) throws MissingProperty {
		double clearanceSqr = clearance * clearance;
		//System.out.println("clearanceSqr=" + clearanceSqr);
		Optional<Robot> failed1;
		Optional<Robot> failed2;
		for (Robot r_i : robots) {
			for (Robot r_j : robots) {
				if (r_i != r_j) {
					Point loc1 = r_i.getPointComponentProperty("location");
					Point loc2 = r_j.getPointComponentProperty("location");
					double distSqr = loc1.distanceSqrTo(loc2);
					System.out.println("loc1 = " + r_i + "loc2 = " + r_j + " dist_sqr = " + distSqr);
					if (distSqr < clearanceSqr) {
						failed1 = Optional.of(r_i);
						failed2 = Optional.of(r_j);
						return Optional.of(Math.sqrt(distSqr));
					}
				}
			}
		}
		return Optional.empty();
 	}
	
	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		List<Robot> robots = mission.getAllRobots();
		try {
			// If the result is present, that means the clearance
			// exceeded the minimum allowed 
			Optional<Double> res = testDistances(robots);
			if (res.isPresent()) {
				Double clearanceFound = res.get();
				// This goal only returns a VIOLATED status, or it returns Empty
				GoalResult gr = new GoalResult(GoalResultStatus.VIOLATED);
				GoalResultField gf = new DoubleResultField("clearance", clearanceFound);
				gr.addField(gf);
				return Optional.of(gr);
			}
		} catch (MissingProperty mp) {
			// TODO: there should be something to signal goal evaluation having failed.
			// In this case, the position properties for a robot are missing
			// For now, return no result
			return Optional.empty();
		}
			
		return Optional.empty();
	}

	protected void setup(Mission mission, Goal g) {
		// TODO Auto-generated method stub
	}
}
