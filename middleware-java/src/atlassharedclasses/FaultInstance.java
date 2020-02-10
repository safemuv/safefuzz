package atlassharedclasses;

import atlasdsl.faults.*;

public class FaultInstance {
	private double startTime;
	private double endTime;
	private Fault fault;
	
	public FaultInstance(Double startTime, Double endTime, Fault f) {
		this.startTime = startTime;
		this.endTime = endTime;
		this.fault = f;
	}

	public boolean isReady(double time) {
		return (time >= startTime) && (time <= endTime);
	}
}
