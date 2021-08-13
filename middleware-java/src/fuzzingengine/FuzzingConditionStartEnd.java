package fuzzingengine;

public class FuzzingConditionStartEnd extends FuzzingTimeSpecification {
	FuzzingCondition startCond;
	FuzzingCondition endCond;
	
	boolean active = false;
	
	double endTime;
	
	public FuzzingConditionStartEnd(FuzzingCondition start, FuzzingCondition end) {
		this.startCond = start;
		this.endCond = end;
	}

	public boolean isActiveAtTime(double time) {
		if (!active) {
			active = startCond.isActive();
		} else {
			active = !endCond.isActive();
		}
		return active;
	}

	public String getCSVContents() {
		return startCond.toString() + "," + endCond.toString();
	}

	protected FuzzingTimeSpecification dup() {
		return new FuzzingConditionStartEnd(startCond.dup(), endCond.dup());
	}
}
