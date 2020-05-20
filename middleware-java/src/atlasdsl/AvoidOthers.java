package atlasdsl;

import java.util.List;
import java.util.Optional;

import atlasdsl.GoalResult.GoalResultStatus;
import atlassharedclasses.Point;
import middleware.core.ATLASCore;

public class AvoidOthers extends GoalAction {
	private double clearance;
	
	public class ViolationRecord {
		private Point loc1;
		private Point loc2;
		private Robot r1;
		private Robot r2;
		private double clearance;
		
		public ViolationRecord(Point loc1, Point loc2, Robot r1, Robot r2, double clearance) {
			this.loc1 = loc1;
			this.loc2 = loc2;
			this.r1 = r1;
			this.r2 = r2;
			this.clearance = clearance;
		}
		
	}
	
	public AvoidOthers(double clearance) {
		this.clearance = clearance;
	}

	// TODO: Specific log for the computation of each goal?
	private Optional<ViolationRecord> testDistances(List<Robot> robots) throws MissingProperty {
		double clearanceSqr = clearance * clearance;
		for (Robot r_i : robots) {
			for (Robot r_j : robots) {
				if (r_i != r_j) {
					Point loc1 = r_i.getPointComponentProperty("location");
					Point loc2 = r_j.getPointComponentProperty("location");
					double distSqr = loc1.distanceSqrTo(loc2);
					//System.out.println("loc1 = " + r_i + "loc2 = " + r_j + " dist_sqr = " + distSqr);
					if (distSqr < clearanceSqr) {
						ViolationRecord vr = new ViolationRecord(loc1, loc2, r_i, r_j);
						return Optional.of(vr);
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
			Optional<Double> vr = testDistances(robots);
			if (vr.isPresent()) {
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

	protected void setup(ATLASCore core, Mission mission, Goal g) {
		// TODO Auto-generated method stub
	}
}
