package atlasdsl;

import atlassharedclasses.Region;

public class DynamicGoalRegion extends GoalRegion {
	private Goal relativeToGoal;
	private String relativeToGoalField;
	private double relativeRange;
	
	public DynamicGoalRegion(Goal relativeToGoal, String relativeToGoalField, double relativeRange) {
		this.relativeToGoal = relativeToGoal;
		this.relativeToGoalField = relativeToGoalField;
		this.relativeRange = relativeRange;
	}

	protected Region getRegion() {
		// TODO: implement dynamic regions lookup
		return null;
	}
}
