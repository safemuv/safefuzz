package atlascollectiveint.api;

public class OneOffTimer extends Timer {
	private double targetTime;
	private boolean cancelled = false;
	
	public OneOffTimer(TimerBehaviour b, double targetTime) {
		super(b);
		this.targetTime = targetTime;
	}

	public void cancel() {
		cancelled = true;
	}

	public boolean isReady(double timeNow) {
		return !cancelled && (timeNow > targetTime);
	}
	
	public boolean shouldRemove(double timeNow) {
		return cancelled || (timeNow > targetTime);
	}
}
