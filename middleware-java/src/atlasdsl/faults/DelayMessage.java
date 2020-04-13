package atlasdsl.faults;

import atlasdsl.Message;
import middleware.core.ATLASCore;

public class DelayMessage extends MessageImpact {
	private double delayLength;

	public DelayMessage(Message affectedMessage, double delayLength) {
		super(affectedMessage);
		this.delayLength = delayLength;
	}
	
	public Object applyImpact(Object orig) {
		return orig;
	}

	public void immediateEffects(ATLASCore core) {
		
	}

	public void completionEffects(ATLASCore core) {
		
	}
	
}
