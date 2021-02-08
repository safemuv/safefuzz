package atlasdsl;

public class GoalActionSetupFailure extends Exception {
	private static final long serialVersionUID = 1L;
	private String reason;
	private GoalAction ga;
	
	GoalActionSetupFailure(GoalAction ga, String reason) {
		this.ga = ga;
		this.reason = reason;
	}
}
