package atlasdsl;

import java.util.List;

import atlassharedclasses.Region;

public abstract class GoalRegion {
	
	protected boolean isDynamic() {
		return false;
	}

	protected abstract List<Region> getRegions();
	protected abstract int getRegionCount();
}
