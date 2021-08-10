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
}
