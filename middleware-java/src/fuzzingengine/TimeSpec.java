package fuzzingengine;

import atlasdsl.Mission;

public class TimeSpec {
	private double startTime;
	private double endTime;
	private Mission mission;
	
	private void checkTiming() throws InvalidTimeSpec {
		if (endTime < startTime) {
			throw new InvalidTimeSpec("endTime < startTime");
		}		
		
		if (startTime < 0.0) {
			throw new InvalidTimeSpec("startTime < 0.0");
		}
		
		if (endTime > mission.getEndTime()) {
			throw new InvalidTimeSpec("endTime > mission end time");
		}
	}
	
	public TimeSpec(Mission mission, double startTime, double endTime) throws InvalidTimeSpec {
		this.mission = mission;
		this.startTime = startTime;
		this.endTime = endTime;
		checkTiming();
	}
	
	public double getLength() {
		return endTime - startTime;
	}
	
	public double getStartTime() {
		return startTime;
	}
	
	public double getEndTime() {
		return endTime;
	}
}