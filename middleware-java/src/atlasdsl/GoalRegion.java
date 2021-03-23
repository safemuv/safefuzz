package atlasdsl;

import java.util.List;
import java.util.Optional;

import atlassharedclasses.Region;

public abstract class GoalRegion {
	
	protected boolean isDynamic() {
		return false;
	}

	protected abstract List<Region> getRegions();
	protected abstract int getRegionCount();
	
	public Optional<Region> getFirstRegion() {
		List<Region> regions = getRegions();
		if (!isDynamic() && regions.size() > 0) {
			return Optional.of(regions.get(0));
		} else {
			return Optional.empty();
		}
	}
}
