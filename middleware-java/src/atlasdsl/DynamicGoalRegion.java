package atlasdsl;

public class DynamicGoalRegion extends GoalRegion {
	private Goal relativeToGoal;
	private String relativeToGoalField;
	private double relativeRange;
	
	public DynamicGoalRegion(Goal relativeToGoal, String relativeToGoalField, double relativeRange) {
		this.relativeToGoal = relativeToGoal;
		this.relativeToGoalField = relativeToGoalField;
		this.relativeRange = relativeRange;
	}
}
