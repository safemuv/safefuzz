package atlasdsl.faults;

import middleware.core.ATLASCore;

public class DelayMessage extends MessageImpact {
	private double delayLength;

	public Object applyImpact(Object orig) {
		return orig;
	}

	public void immediateEffects(ATLASCore core) {
		
	}
}
