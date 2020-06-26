package exptrunner;

import java.util.LinkedHashMap;
import java.util.Map;

public class ResultInfo {
	
	private Map<String,Integer> fields = new LinkedHashMap<String,Integer>();
	
	public int getTotalGoalViolations() {
		int total = 0;
		for (Map.Entry<String, Integer> me : fields.entrySet()) {
			total += me.getValue();
		}
		return total;
	}
	
	public void setField(String key, int val) {
		fields.put(key, val);
	}
}
