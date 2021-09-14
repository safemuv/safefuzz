package fuzzingengine;

public class FuzzingFixedTimeSpecification extends FuzzingTimeSpecification {
	protected double startTime;
	protected double endTime;
	
	public FuzzingFixedTimeSpecification(double startTime, double endTime) {
		this.startTime = startTime;
		this.endTime = endTime;
	}
	
	public boolean isActiveAtTime(double time, String vehicle) {
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
	
	public String getCSVRecordTag() {
		return "KEY";
	}

	public void validateSpecification() throws InvalidSpecification {
		if (endTime < startTime) {
			throw new InvalidSpecification("End time is not greater than start time in " + this.toString());
		}
	}
}
