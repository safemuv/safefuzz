package atlasdsl;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class GoalResult {
	List<GoalResultField> goalResults = new ArrayList<GoalResultField>();
	
	public GoalResult() {
		
	}
	
	public void addField(GoalResultField gf) {
		goalResults.add(gf);
	}
	
	public String toString() {
		List<String> res = goalResults.stream().map(Object::toString).collect(Collectors.toList());
		return String.join(",", res);
	}
}
