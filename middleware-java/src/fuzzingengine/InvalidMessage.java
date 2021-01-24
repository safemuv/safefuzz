package fuzzingengine;

public class InvalidMessage extends Exception {

	private static final long serialVersionUID = 1L;
	private String messageName;
	private String reason;
	
	public InvalidMessage(String messageName, String reason) {
		this.messageName = messageName;
		this.reason = reason;
	}
	
	public String getMessageName() {
		return messageName;
	}
	
	public String getReason() {
		return reason;
	}
}
