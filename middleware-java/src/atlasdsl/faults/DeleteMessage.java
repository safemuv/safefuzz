package atlasdsl.faults;

import atlasdsl.Message;
import middleware.core.ATLASCore;

public class DeleteMessage extends MessageImpact {
	public Object applyImpact(Object orig) {
		return orig;
	}
	
	public DeleteMessage(Message msg) {
		super(msg);
	}

	public void immediateEffects(ATLASCore core) {
	
	}
}
