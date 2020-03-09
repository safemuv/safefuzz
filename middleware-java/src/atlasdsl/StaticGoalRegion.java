package atlasdsl;

import java.util.ArrayList;
import java.util.List;

import atlassharedclasses.Region;

public class StaticGoalRegion extends GoalRegion {
	private Region region;
	private List<Region> regionList;
	
	public StaticGoalRegion(Region region) {
		this.region = region;
		this.regionList = new ArrayList<Region>();
		regionList.add(region);
	}
	
	protected Region getRegion() {
		return region;
	}
	
	protected boolean isDynamic() {
		return false;
	}

	@Override
	protected List<Region> getRegions() {
		return regionList;
	}

	@Override
	protected int getRegionCount() {
		// TODO Auto-generated method stub
		return 0;
	}
}
