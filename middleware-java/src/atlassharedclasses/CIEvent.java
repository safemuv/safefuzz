package atlassharedclasses;

import com.fasterxml.jackson.annotation.JsonProperty;

public class CIEvent {
	@JsonProperty("command")
	private BehaviourCommand command;
	private String robotName;
	
	public CIEvent() {
		
	}
	
	public CIEvent(BehaviourCommand cmd, String robotName) {
		this.command = cmd;
		this.robotName = robotName;
	}
	
	public BehaviourCommand getCommand() {
		return command;
	}
	
	public String getRobotName() {
		return robotName;
	}
}
