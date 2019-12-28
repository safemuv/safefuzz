package atlasdsl;

import java.util.ArrayList;
import java.util.List;

public class Mission {
	private List<Component> participants;
	
	public List<Robot> getAllRobots() {
		return new ArrayList<Robot>();
	}
	
	public boolean includesComputer() {
		return true;
	}
}
