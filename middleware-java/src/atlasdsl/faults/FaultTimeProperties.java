package atlasdsl.faults;

public class FaultTimeProperties {
	private double earliestFaultStart;
	private double latestFaultEnd;
	private double maxFaultLength;
	private int maxFaultRepeatCount;
	
	public FaultTimeProperties(double faultStart, double faultEnd, double faultLength, int faultRepeatCount) {	
		this.earliestFaultStart = faultStart;
		this.latestFaultEnd = faultEnd;
		this.maxFaultLength = faultLength;
		this.maxFaultRepeatCount = faultRepeatCount;
	}
	
	public boolean isInRange(double start, double length) {
		double end = start + length;
		return (start >= earliestFaultStart) && (end <= latestFaultEnd);
	}
	
	public int getMaxRepeatCount() {
		return maxFaultRepeatCount;
	}
	
	public double getEarliestStart() {
		return earliestFaultStart;
	}
	
	public double getLatestEnd() {
		return latestFaultEnd;
	}
	
	public double getMaxFaultLength() {
		return maxFaultLength;
	}
}