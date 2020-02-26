package atlascollectiveint.api;

public class PeriodicTimer extends Timer {
	private double period;
	private double nextTime;
	private boolean cancelled = false;
	
	public PeriodicTimer(double period, TimerBehaviour b) {
		super(b);
		this.period = period;
		this.nextTime = period;
	}
	
	public void cancel() {
		cancelled = true;
	}

	public boolean isReady(double timeNow) {
		return !cancelled && (timeNow > nextTime);
	}

	public boolean shouldRemove(double timeNow) {
		return cancelled;
	}
	
	public void performAction() {
		super.performAction();
		nextTime += period;
	}
}