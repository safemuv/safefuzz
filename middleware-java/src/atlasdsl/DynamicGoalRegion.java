package atlasdsl;

import java.util.List;

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

	protected List<Region> getRegions() {
		// TODO: implement dynamic regions lookup from a 
		// database of previous goal statuses
		return null;
	}
	
	protected int getRegionCount() {
		return 0;
	}
	
	protected boolean isDynamic() {
		return true;
	}
}
