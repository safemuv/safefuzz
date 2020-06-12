package atlassharedclasses;

public class SetDepth extends BehaviourCommand {
	private double depth;
	
	SetDepth() {
		
	}
	
	public SetDepth(double depth, String messageName) {
		super(messageName);
		this.depth = depth;

	}
		
	public double getDepth() {
		return depth;
	}
}
