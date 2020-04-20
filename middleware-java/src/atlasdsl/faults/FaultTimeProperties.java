package atlasdsl.faults;

public class FaultTimeProperties {
	private double earliestFaultStart;
	private double latestFaultEnd;
	private double maxFaultLength;
	private int maxFaultRepeatCount;
	private double faultProb;
	
	public FaultTimeProperties(double faultStart, double faultEnd, double faultLength, int faultRepeatCount, double faultProb) {	
		this.earliestFaultStart = faultStart;
		this.latestFaultEnd = faultEnd;
		this.maxFaultLength = faultLength;
		this.maxFaultRepeatCount = faultRepeatCount;
		this.faultProb = faultProb;
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
	
	public double getFaultProb() {
		return faultProb;
	}
}