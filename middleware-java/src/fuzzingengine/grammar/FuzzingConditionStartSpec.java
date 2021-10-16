package fuzzingengine.grammar;

import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.InvalidCondition;
import fuzzingengine.InvalidSpecification;

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
	
	public boolean isActiveAtTime(double time, String vehicle) {
		if (!started) {
			started = startCond.evaluate(vehicle);
		} else {
			started = (time < endTime);
		}
		return started;
	}

	public String getCSVContents() {
		return startCond.jsonPrint() + "," + endTime;
	}

	protected FuzzingTimeSpecification dup() {
		return new FuzzingConditionStartSpec(startCond, endTime);
	}

	public String getCSVRecordTag() {
		return "KEYCONDSTART";
	}

	public void validateSpecification() throws InvalidSpecification {
		try {
			startCond.validateCondition();
		} catch (InvalidCondition e) {
			throw new InvalidSpecification(e);
		}
	}
}