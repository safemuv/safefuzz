package atlasdsl;

import java.util.ArrayList;
import java.util.List;

public class Mission {
	private List<Robot> robots = new ArrayList<Robot>();
	private List<Computer> computers = new ArrayList<Computer>();
	private List<EnvironmentalObject> objects = new ArrayList<EnvironmentalObject>();
	
	public List<Robot> getAllRobots() {
		return robots;
	}
	
	public boolean includesComputer() {
		return (computers.size() > 0); 
	}
	
	public void addRobot(Robot r) {
		robots.add(r);
	}
	
	//TODO: setters for the other geometric regions as well
	public void addObject(EnvironmentalObject eo) {
		objects.add(eo);
	}
	
	public void addComputer(Computer c) {
		computers.add(c);
	}

	public List<EnvironmentalObject> getEnvironmentalObjects() {
		return objects;
	}
}
