package fuzzingengine;

import middleware.core.ATLASCore;

public class FuzzingConditionStartSpec extends FuzzingTimeSpecification {
	FuzzingCondition startCond;
	
	boolean started = false;
	
	double endTime;
	
	public FuzzingConditionStartSpec(FuzzingCondition startCond, double endTime) {
		this.startCond = startCond.dup();
		this.endTime = endTime;
	}
	
	public FuzzingCondition getCondition() {
		return startCond;
	}
	
	public double getEndTime() {
		return endTime;
	}
	
	public boolean isActiveAtTime(double time) {
		if (!started) {
			started = startCond.evaluate();
		} else {
			started = (time < endTime);
		}
		return started;
	}

	public String getCSVContents() {
		return startCond.csvPrint() + "," + endTime;
	}

	protected FuzzingTimeSpecification dup() {
		return new FuzzingConditionStartSpec(startCond, endTime);
	}

	public String getCSVRecordTag() {
		return "KEYCONDSTART";
	}
}