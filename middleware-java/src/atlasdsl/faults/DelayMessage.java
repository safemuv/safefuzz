package atlasdsl.faults;

public class DelayMessage extends MessageImpact {
	private double delayLength;

	public Object applyImpact(Object orig) {
		return orig;
	}
}
