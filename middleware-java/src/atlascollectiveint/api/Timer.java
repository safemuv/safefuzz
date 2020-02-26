package atlascollectiveint.api;

public abstract class Timer {
	TimerBehaviour b;
	
	public Timer(TimerBehaviour b) {
		this.b = b;
	}
	
	public abstract void cancel();
	public abstract boolean isReady(double timeNow);
	public abstract boolean shouldRemove(double timeNow);
	
	public void performAction() {
		b.op(this);
	}
}
