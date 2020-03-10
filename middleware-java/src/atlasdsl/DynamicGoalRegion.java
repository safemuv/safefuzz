package atlasdsl;

import java.util.ArrayList;
import java.util.List;

import atlassharedclasses.*;

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
		List<GoalResult> gres = relativeToGoal.getResults();
		
		List<Region> res = new ArrayList<Region>();
		for (GoalResult gr : gres) {
			GoalResultField gf = gr.getField(relativeToGoalField);
			if (gf instanceof PointResultField) {
				PointResultField pf = (PointResultField)gf;
				Point detectionPoint = pf.getValue();
				Region r = Region.squareAroundPoint(detectionPoint, relativeRange * 2.0);
				res.add(r);
			}
		}
		return res;
	}
	
	protected int getRegionCount() {
		// The number of regions is one per goal result obtained
		return relativeToGoal.getResults().size();
	}
	
	protected boolean isDynamic() {
		return true;
	}
}
