package atlasdsl;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import atlasdsl.GoalResult.GoalResultStatus;
import atlassharedclasses.Point;
import atlassharedclasses.Region;
import middleware.core.ATLASCore;
import middleware.logging.ATLASLog;

// Assume StayInRegion requires a staticGoalRegion currently
public class StayInRegion extends GoalAction {
	private Region region;
	private ATLASCore core;
	private boolean stopOnFirstViolation = false;
	
	private int violationIncidents = 0;
	private final double TIME_THRESHOLD = 1.0;
	
	private HashMap<String,Double> robotLastViolationTime = new HashMap<String,Double>();
	
	public class MissingRegion extends Exception {
		private static final long serialVersionUID = 1L;
	}

	public StayInRegion() {
		// The region is determined during the setup method
	}
	
	public StayInRegion(boolean stopOnFirstViolation) {
		this.stopOnFirstViolation = stopOnFirstViolation;
	}
	
	public boolean violationReadyToLog(String robotName, double currentTime) {
		if (!robotLastViolationTime.containsKey(robotName)) {
			robotLastViolationTime.put(robotName, Double.MIN_VALUE);
		}
		
		double lastTime = robotLastViolationTime.get(robotName);
		if ((currentTime - lastTime) >= TIME_THRESHOLD) {
			robotLastViolationTime.put(robotName, currentTime);
			return true;
		} else {
			return false;
		}
	}
	
	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		List<Robot> robots = participants.getParticipants();
		try {
			for (Robot r : robots) {
				Point loc = r.getPointComponentProperty("location");
				System.out.println("StayInRegion: " + r.getName() + " is at " + loc);
				if (!region.contains(loc)) {
					double time = core.getTime();
					if (violationReadyToLog(r.getName(), time)) {
						violationIncidents++;
					
						GoalResultStatus grs;
						if (stopOnFirstViolation) {
							grs = GoalResultStatus.VIOLATED;
						} else {
							grs = GoalResultStatus.CONTINUE;
						}
										
						ATLASLog.logGoalMessage(this, violationIncidents + "," + time + "," + r.getName() + "," + loc);
						GoalResult gr = new GoalResult(grs);
						GoalResultField gf = new StringResultField("violatingRobot", r.getName());
						GoalResultField pf = new PointResultField("location", loc);
						gr.addField(gf);
						gr.addField(pf);
						return Optional.of(gr);
					}
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
		this.core = core;
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
