package fuzzingengine.grammar;

import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.InvalidCondition;
import fuzzingengine.InvalidSpecification;
import middleware.core.ATLASCore;

public class FuzzingConditionStartEnd extends FuzzingTimeSpecification {
	FuzzingCondition startCond;
	FuzzingCondition endCond;
	
	boolean active = false;
	
	double endTime;
	
	public FuzzingConditionStartEnd(FuzzingCondition start, FuzzingCondition end) {
		this.startCond = start;
		this.endCond = end;
	}

	public boolean isActiveAtTime(double time, String vehicle) {
		if (!active) {
			active = startCond.evaluate(vehicle);
		} else {
			active = !endCond.evaluate(vehicle);
		}
		return active;
	}

	public String getCSVContents() {
		return startCond.jsonPrint() + "," + endCond.jsonPrint();
	}

	protected FuzzingTimeSpecification dup() {
		return new FuzzingConditionStartEnd(startCond.dup(), endCond.dup());
	}
	
	public String getCSVRecordTag() {
		return "KEYCONDBOTH";
	}

	public void validateSpecification() throws InvalidSpecification {
		try {
			startCond.validateCondition();
			endCond.validateCondition();
		} catch (InvalidCondition e) {
			e.printStackTrace();
		}
	}
	
	public FuzzingCondition getStartCondition() {
		return startCond;
	}
	
	public FuzzingCondition getEndCondition() {
		return endCond;
	}
}
