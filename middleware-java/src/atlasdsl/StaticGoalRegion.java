package atlasdsl;

import atlassharedclasses.Region;

public class StaticGoalRegion extends GoalRegion {
	private Region region;
	
	public StaticGoalRegion(Region region) {
		this.region = region;
	}
	
	protected Region getRegion() {
		return region;
	}
}
