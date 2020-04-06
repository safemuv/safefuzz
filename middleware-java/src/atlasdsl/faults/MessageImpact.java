package atlasdsl.faults;

import atlasdsl.*;
import middleware.core.ATLASCore;

public abstract class MessageImpact extends FaultImpact {
	private Message affectedMessage;
	
	public MessageImpact(Message affectedMessage) {
		this.affectedMessage = affectedMessage;
	}
}
