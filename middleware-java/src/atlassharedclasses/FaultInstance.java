package atlassharedclasses;

import java.util.Optional;

import atlasdsl.faults.*;

public class FaultInstance implements Comparable<FaultInstance> {
	private double startTime;
	private double endTime;
	// Fault instances are always active by default - for compatibility
	private boolean isActive = true;
	private Fault fault;
	private Optional<String> extraData;
	
	public FaultInstance(Double startTime, Double endTime, Fault f, Optional<String> extraData) {
		//System.out.println("startTime = " + startTime + ",endTime = " + endTime);
		this.startTime = startTime;
		this.endTime = endTime;
		this.fault = f;
		this.extraData = extraData;		
		//System.out.println("FaultInstance extraData = " + extraData);
		
		// hack to change the speed for MotionFault - for paper experiments only
		//if (f.getImpact() instanceof MotionFault && extraData.isPresent()) {
//			double speedOverride = Double.valueOf(extraData.get());
			//MotionFault mfi = (MotionFault)f.getImpact();
			//System.out.println("Experiment overriding speed to " + speedOverride);
			//mfi._overrideSpeed(speedOverride);
			// test
			//MotionFault mfi2 = (MotionFault)f.getImpact();
			//System.out.println("test newValue changed = " + mfi2.getNewValue());
		//}
	}
	
	public void setActiveFlag(boolean flag) {
		this.isActive = flag;
	}
	
	public FaultInstance(FaultInstance orig) {
		this.endTime = orig.endTime;
		this.startTime = orig.startTime;
		this.fault = orig.fault;
		this.extraData = orig.extraData;
	}
	
	public int compareTo(FaultInstance other) {
		int timeCompare = Double.compare(this.startTime, other.startTime);
		if (timeCompare != 0) {
			return timeCompare;
		} else {
			int idCompare = this.fault.getName().compareTo(other.fault.getName());
			return idCompare;
		}
	}
	
	public String toString() {
		String isActiveStr = "";
		if (!isActive) {
			isActiveStr = ",[INACTIVE]";
		}
		return fault.getName() + "," + startTime + "," + endTime + "," + extraData + isActiveStr; 
	}

	public boolean isReady(double time) {
		return (time >= startTime) && (time <= endTime);
	}
	
	public Fault getFault() {
		return fault;
	}
	
	public boolean isValid() {
		FaultTimeProperties ftp = fault.getTimeProperties();
		return ftp.isInRange(startTime, endTime);
	}

	public boolean isFinished(double time) {
		return (time > endTime);
	}

	public double getEndTime() {
		return endTime;
	}

	public double getStartTime() {
		return startTime;
	}
	
	public double getLength() { 
		return endTime - startTime;
	}
	
	
	public String getExtraData() {
		if (extraData.isPresent()) {
			return extraData.get();
		}	else { 
			return "";
		}
	}
	
	public Optional<String> getExtraDataOpt() {
		return extraData;
	}
	
	private void constrainTimeValid() {
		Fault f = this.getFault();
		// Check if corresponding to the length of the fault in the model
		if (this.endTime > f.getLatestEndTime()) {
			this.endTime = f.getLatestEndTime();
		}
		
		if (this.endTime < f.getEarliestStartTime()) {
			this.endTime = f.getEarliestStartTime();
		}
		
		if (this.startTime < f.getEarliestStartTime()) {
			this.startTime = f.getEarliestStartTime();
		}
		
		if (this.startTime > f.getLatestEndTime()) {
			this.startTime = f.getLatestEndTime();
		}
	}

	public void multLengthFactor(double factor) {
		double origLen = endTime - startTime;
		System.out.println("origLen = " + origLen);
		endTime = startTime + (origLen * factor);
		
		System.out.println("endTime=" + endTime);
		constrainTimeValid();
	}

	public void absShiftTimes(double absTimeShift) {
		Fault f = this.getFault();
		this.startTime += absTimeShift;
		this.endTime += absTimeShift;
		constrainTimeValid();
	}

	public boolean isActive() {
		return isActive;
	}

	public void flipActiveFlag() {
		isActive = !isActive;
	}

	public void setExtraData(String string) {
		extraData = Optional.of(string);
	}
}
