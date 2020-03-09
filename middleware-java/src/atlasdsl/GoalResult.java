package atlasdsl;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class GoalResult {
	public enum GoalResultStatus {
		COMPLETED,
		VIOLATED,
		CONTINUE
	}
	
	private GoalResultStatus status;
	
	Map<String,GoalResultField> goalResults = new LinkedHashMap<String,GoalResultField>();
	
	
	public GoalResult(GoalResultStatus status) {
		this.status = status;
	}
	
	public GoalResultStatus getResultStatus() {
		return status;
	}
	
	public void addField(GoalResultField gf) {
		goalResults.put(gf.name, gf);
	}
	
	public GoalResultField getField(String fieldName) {
		return goalResults.get(fieldName);
	}
	
	public String toString() {
		return status.toString() + " - " + goalResults.toString();
		//List<String> res = goalResults.entrySet().map(Object::toString).collect(Collectors.toList());
		//return String.join(",", res);
	}
}
