package fuzzingengine;

import java.util.Optional;

public class FuzzingFixedTimeSpecification extends FuzzingTimeSpecification {
	protected double startTime;
	protected double endTime;
	
	public FuzzingFixedTimeSpecification(double startTime, double endTime) {
		this.startTime = startTime;
		this.endTime = endTime;
	}
	
	public FuzzingFixedTimeSpecification(FuzzingFixedTimeSpecification other) {
		this.startTime = other.startTime;
		this.endTime = other.endTime;
	}
	
	public boolean isActiveAtTime(double time, String vehicle) {
		return (time >= startTime) && (time < endTime);
	}

	public String getCSVContents() {
		return startTime + "," + endTime;
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
	
	public Optional<Double> getStaticLength() {
		return Optional.of(endTime - startTime);
	}

	protected FuzzingTimeSpecification dup() {
		return new FuzzingFixedTimeSpecification(startTime, endTime);
	}
	
	public String getCSVRecordTag() {
		return "KEY";
	}

	public void validateSpecification() throws InvalidSpecification {
		if (endTime < startTime) {
			throw new InvalidSpecification("End time is not greater than start time in " + this.toString());
		}
	}
}
