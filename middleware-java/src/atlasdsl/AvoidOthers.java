package atlasdsl;

import java.util.List;
import java.util.Optional;

import atlassharedclasses.Point;

public class AvoidOthers extends GoalAction {
	private double clearance;
	
	public AvoidOthers(double clearance) {
		this.clearance = clearance;
	}

	private Optional<Double> testDistances(List<Robot> robots) throws MissingProperty {
		double clearanceSqr = clearance * clearance;
		System.out.println("clearanceSqr=" + clearanceSqr);
		Optional<Robot> failed1;
		Optional<Robot> failed2;
		for (Robot r_i : robots) {
			for (Robot r_j : robots) {
				if (r_i != r_j) {
					Point loc1 = r_i.getPointComponentProperty("location");
					Point loc2 = r_j.getPointComponentProperty("location");
					double dist = loc1.distanceSqrTo(loc2);
					System.out.println("loc1 = " + r_i + "loc2 = " + r_j + " dist_sqr = " + dist);
					if (dist < clearanceSqr) {
						failed1 = Optional.of(r_i);
						failed2 = Optional.of(r_j);
						return Optional.of(dist);
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
				GoalResult gr = new GoalResult();
				GoalResultField gf = new DoubleResultField("clearance", clearanceFound);
				gr.addField(gf);
				return Optional.of(gr);
			}
		} catch (MissingProperty mp) {
			// TODO: there should be something to signal goal evaluation having failed.
			// In this case, there are no position properties for a robot.
			// For now, return no result
			return Optional.empty();
		}
			
		return Optional.empty();
	}

	protected void setup(Mission mission, Goal g) {
		// TODO Auto-generated method stub
	}
}
