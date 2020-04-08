package atlasdsl.faults;

import atlasdsl.*;

public abstract class MessageImpact extends FaultImpact {
	private Message affectedMessage;
	private int fromHash;
	private int toHash;
	
	public MessageImpact(Message affectedMessage) {
		this.affectedMessage = affectedMessage;
		this.fromHash = affectedMessage.getFrom().hashCode();
		this.toHash = affectedMessage.getTo().hashCode();
	}
	
	public Message getMessage() {
		return affectedMessage;
	}
	
	public boolean matchesNamedComponents(Component to, Component from) {
		return (to.hashCode() == toHash) && (from.hashCode() == fromHash);
	}
	
	public boolean matchesComponentTo(Component to) {
		return (to.hashCode() == toHash);
	}
}
