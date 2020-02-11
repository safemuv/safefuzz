package atlassharedclasses;

import java.util.List;

public class SetCoordinates extends BehaviourCommand {
	private List<Point> coordinates;
	
	SetCoordinates() {
		
	}
	
	public SetCoordinates(List<Point> coordinates) {
		this.coordinates = coordinates;
	}
		
	public List<Point> getCoordinates() {
		return coordinates;
	}
}