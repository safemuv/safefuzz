package atlascollectiveint.api;

import atlascollectiveint.logging.CollectiveIntLog;
import atlascollectiveintgenerator.CollectiveInt;

public class OneOffTimer extends Timer {
	private double targetTime;
	private boolean cancelled = false;
	
	public OneOffTimer(TimerBehaviour b, double targetTime) {
		super(b);
		this.targetTime = targetTime;
	}
	
	public static OneOffTimer atTime(double targetTime, TimerBehaviour b) {
		return new OneOffTimer(b, targetTime);
	}
	
	public static OneOffTimer afterDelay(double delay, TimerBehaviour b) {
		double targetTime = CollectiveInt.getTimeNow() + delay;
		CollectiveIntLog.logCI("Creating new OneOffTimer: targetTime = " + targetTime);
		System.out.println("Creating new OneOffTimer: targetTime = " + targetTime);
		return new OneOffTimer(b, targetTime);
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
