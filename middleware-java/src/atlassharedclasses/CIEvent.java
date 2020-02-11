package atlassharedclasses;

public class CIEvent {
	private BehaviourCommand cmd;
	private String robotName;
	
	public CIEvent() {
		
	}
	
	public CIEvent(BehaviourCommand cmd, String robotName) {
		this.cmd = cmd;
		this.robotName = robotName;
	}
	
	public BehaviourCommand getCommand() {
		return cmd;
	}
	
	public String getRobotName() {
		return robotName;
	}
}
