package atlascollectiveint.api;

public abstract class Timer {
	TimerBehaviour b;
	
	public Timer(TimerBehaviour b) {
		this.b = b;
	}
	
	public abstract void cancel();
}
