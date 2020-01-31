package atlascollectiveint.api;

public class PeriodicTimer extends Timer {
	private double period;
	
	public PeriodicTimer(double period, TimerBehaviour b) {
		super(b);
		this.period = period;
	}
	
	public void cancel() {
		
	}
}