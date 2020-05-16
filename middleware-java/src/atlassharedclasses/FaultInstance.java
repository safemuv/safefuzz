package atlassharedclasses;

import java.util.Optional;

import atlasdsl.faults.*;

public class FaultInstance {
	private double startTime;
	private double endTime;
	private Fault fault;
	private Optional<String> extraData;
	
	public FaultInstance(Double startTime, Double endTime, Fault f, Optional<String> extraData) {
		System.out.println("startTime = " + startTime + ",endTime = " + endTime);
		this.startTime = startTime;
		this.endTime = endTime;
		this.fault = f;
		this.extraData = extraData;		
		System.out.println("FaultInstance extraData = " + extraData);
		
		// hack to change the speed for MotionFault - for paper experiments only
		if (f.getImpact() instanceof MotionFault && extraData.isPresent()) {
			double speedOverride = Double.valueOf(extraData.get());
			MotionFault mfi = (MotionFault)f.getImpact();
			System.out.println("Experiment overriding speed to " + speedOverride);
			mfi._overrideSpeed(speedOverride);
			// test
			MotionFault mfi2 = (MotionFault)f.getImpact();
			System.out.println("test newValue changed = " + mfi2.getNewValue());
		}
	}
	
	public String toString() {
		return fault.toString() + "," + startTime + "," + endTime; 
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

	public String getExtraData() {
		if (extraData.isPresent()) {
			return extraData.get();
		}	else { 
			return "";
		}
	}
}
