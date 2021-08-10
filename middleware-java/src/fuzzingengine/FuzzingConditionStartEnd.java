package fuzzingengine;

public class FuzzingConditionStartEnd extends FuzzingTimeSpecification {
	FuzzingCondition startCond;
	FuzzingCondition endCond;
	
	boolean active = false;
	
	double endTime;
	
	public boolean isActiveAtTime(double time) {
		if (!active) {
			active = startCond.isActive();
		} else {
			active = !endCond.isActive();
		}
		return active;
	}
}
