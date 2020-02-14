package atlasdsl.faults;

import middleware.core.ATLASCore;

public class DeleteMessage extends MessageImpact {
	public Object applyImpact(Object orig) {
		return orig;
	}

	public void immediateEffects(ATLASCore core) {
	
	}
}
