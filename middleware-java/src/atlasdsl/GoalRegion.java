package atlasdsl;

import atlassharedclasses.Region;

public abstract class GoalRegion {
	protected abstract Region getRegion();
	
	protected boolean isDynamic() {
		return false;
	}
}
