package fuzzingengine;

import fuzzingengine.operations.FuzzingOperation;

abstract public class FuzzingSelectionRecord {
	protected FuzzingTimeSpecification timeSpec;
	protected boolean isActive;
	protected FuzzingOperation op;
	
	public FuzzingSelectionRecord(FuzzingOperation op) {
		this.op = op;
	}
	
	protected FuzzingOperation getOperation() {
		return op;
	}
	
	public boolean isActiveAtTime(double time, String vehicle) {
		return timeSpec.isActiveAtTime(time, vehicle);
	}
	
	public abstract FuzzingSelectionRecord dup();

	public void absShiftTimes(double absTimeShift) {
		
	}
	
	public boolean isActive() {
		return isActive;
	}

	public abstract String generateCSVLine();

	public abstract void checkConstraints();
	
	public abstract String getSimpleName();
	
	protected String generateOpParams() {
		return "";
	}
	
	public FuzzingTimeSpecification getTimeSpec() {
		return timeSpec;
	}
	
	public void setTimeSpec(FuzzingTimeSpecification ts) {
		this.timeSpec = ts;
	}

	public boolean containsTopic(String key) {
		return false;
	}
}
