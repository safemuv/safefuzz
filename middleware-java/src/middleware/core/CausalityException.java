package middleware.core;

public class CausalityException extends Exception {
	private double timeNew;
	private double originalTime;
	
	private static final long serialVersionUID = 1L;

	public CausalityException(double timeNew, double originalTime) {
		this.timeNew = timeNew;
		this.originalTime = originalTime;
	}
	
	public double getNewTime() {
		return timeNew;
	}
	
	public double getOriginalTime() {
		return originalTime;
	}
	
	public double timeDiff() {
		return timeNew - originalTime;
	}
}
