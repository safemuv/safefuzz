package atlassharedclasses;

import java.util.List;

public class SetCoordinates extends BehaviourCommand {
	private List<Point> coordinates;
	private int repeatCount;
	
	SetCoordinates() {
		
	}
	
	public SetCoordinates(List<Point> coordinates, int repeatCount, String messageName) {
		super(messageName);
		this.coordinates = coordinates;

	}
		
	public List<Point> getCoordinates() {
		return coordinates;
	}
	
	public int getRepeatCount() {
		return repeatCount;
	}
}