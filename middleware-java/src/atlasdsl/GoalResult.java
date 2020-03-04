package atlasdsl;

import java.util.ArrayList;
import java.util.List;

public class GoalResult {
	List<GoalResultField> goalResults = new ArrayList<GoalResultField>();
	
	public GoalResult() {
		
	}
	
	public void addField(GoalResultField gf) {
		goalResults.add(gf);
	}
	
}
