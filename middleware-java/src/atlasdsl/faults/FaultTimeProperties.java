package atlasdsl.faults;

public class FaultTimeProperties {
	private double faultStart;
	private double faultLength;
	private double faultPeriod;
	private int faultRepeatCount;
	
	public FaultTimeProperties(double faultStart, double faultLength, double faultPeriod, int faultRepeatCount) {
		this.faultStart = faultStart;
		this.faultLength = faultLength;
		this.faultPeriod = faultPeriod;
		this.faultRepeatCount = faultRepeatCount;
	}
}