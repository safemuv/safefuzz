package atlassharedclasses;

import java.util.List;

public class SetCoordinates extends BehaviourCommand {
	SetCoordinates() {
	}
	
	public SetCoordinates(List<Point> coordinates) {
		this.coordinates = coordinates;
	}
	
	private List<Point> coordinates;
}
