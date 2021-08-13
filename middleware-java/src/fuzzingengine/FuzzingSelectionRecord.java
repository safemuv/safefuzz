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
	
//	public double getStartTime() {
//		return startTime;
//	}
//	
//	public void setStartTime(double startTime) {
//		this.startTime = startTime;
//	}
//	
//	public void setEndTime(double endTime) {
//		this.endTime = endTime;
//	}
//	
//	public double getEndTime() {
//		return endTime;
//	}
	
	public boolean isReadyAtTime(double time) {
		return timeSpec.isActiveAtTime(time);
	}
	
	public abstract FuzzingSelectionRecord dup();

	public void absShiftTimes(double absTimeShift) {
		
	}
	
	public boolean isActive() {
		return isActive;
	}

	public abstract String generateCSVLine();

	public abstract void checkConstraints();
	
	protected String generateOpParams() {
		return "";
	}
	
	public FuzzingTimeSpecification getTimeSpec() {
		return timeSpec;
	}

}
