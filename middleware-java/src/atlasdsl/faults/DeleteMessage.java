package atlasdsl.faults;

import java.util.Optional;

import atlasdsl.Message;
import middleware.core.ATLASCore;

public class DeleteMessage extends MessageImpact {
	public Object applyImpact(Object orig, Optional<String> additionalData) {
		return orig;
	}
	
	public DeleteMessage(Message msg) {
		super(msg);
	}

	public void immediateEffects(ATLASCore core, Optional<String> additionalData) {
	
	}

	public void completionEffects(ATLASCore core, Optional<String> additionalData) {
		
	}
}
