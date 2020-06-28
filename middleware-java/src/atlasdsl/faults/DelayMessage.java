package atlasdsl.faults;

import java.util.Optional;

import atlasdsl.Message;
import middleware.core.ATLASCore;

public class DelayMessage extends MessageImpact {
	private double delayLength;

	public DelayMessage(Message affectedMessage, double delayLength) {
		super(affectedMessage);
		this.delayLength = delayLength;
	}
	
	public Object applyImpact(Object orig, Optional<String> additionalData) {
		return orig;
	}

	public void immediateEffects(ATLASCore core, Optional<String> additionalData) {
		
	}

	public void completionEffects(ATLASCore core, Optional<String> additionalData) {
		
	}
}
