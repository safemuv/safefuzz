package fuzzingengine.grammar;

import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.InvalidCondition;
import fuzzingengine.InvalidSpecification;
import middleware.core.ATLASCore;
import middleware.logging.ATLASLog;

public class FuzzingConditionStartEnd extends FuzzingTimeSpecification {
	FuzzingCondition startCond;
	FuzzingCondition endCond;
	
	boolean active = false;
	int activationCount = 0;
	int maxActivationCount = 1;
	
	double endTime;
	
	public FuzzingConditionStartEnd(FuzzingCondition start, FuzzingCondition end) {
		this.startCond = start;
		this.endCond = end;
	}

	public boolean isActiveAtTime(double time, String vehicle) {
		if (!active && (activationCount < maxActivationCount)) {
			if (startCond.evaluate(vehicle)) {
				active = true;
				activationCount++;
				ATLASLog.logFuzzing("Fuzzing condition startCond true: activating fuzzing on " + this);
			}
		}
		
		if (active) {
			// Active is always true here
			// When endCond evaluates to true, reset active to false 
			ATLASLog.logFuzzing("Fuzzing condition endCond true: deactivating fuzzing on " + this);
			active = !endCond.evaluate(vehicle);
		}
		
		// TODO: Need to check active and log the total number of second intervals and conditions are evaluating for true to...
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
