package fuzzingengine;

public class FuzzingFixedTimeSpecification extends FuzzingTimeSpecification {
	protected double startTime;
	protected double endTime;
	
	public FuzzingFixedTimeSpecification(double startTime, double endTime) {
		this.startTime = startTime;
		this.endTime = endTime;
	}
	
	public boolean isActiveAtTime(double time) {
		return (time >= startTime) && (time < endTime);
	}

	public String getCSVContents() {
		return null;
	}

	public double getStartTime() {
		return startTime;
	}
	
	public double getEndTime() {
		return endTime;
	}
	
	public void setStartTime(double startTime) {
		this.startTime = startTime;
	}
	
	public void setEndTime(double endTime) {
		this.endTime = endTime;
	}

	protected FuzzingTimeSpecification dup() {
		return new FuzzingFixedTimeSpecification(startTime, endTime);
	}
}
