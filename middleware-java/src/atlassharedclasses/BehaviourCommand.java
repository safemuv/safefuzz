package atlassharedclasses;

public abstract class BehaviourCommand {
	// This references the associated Message with this command
	private String messageName;
	
	public String getMessageName() {
		return messageName;
	}
}
