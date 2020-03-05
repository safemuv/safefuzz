package atlasdsl;

import java.util.ArrayList;
import java.util.List;

public class RobotResultField extends GoalResultField {
	public List<Robot> robots;
	
	RobotResultField(List<Robot> robots) {
		this.robots = robots;
	}
	
	RobotResultField(Robot singleRobot) {
		robots = new ArrayList<Robot>();
		robots.add(singleRobot);
	}

	public List<Robot> getRobots() {
		return robots;
	}
	
	public String toString() {
		return this.name + "=" + robots.toString();
	}
}
