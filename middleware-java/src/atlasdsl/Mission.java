package atlasdsl;

import atlasdsl.*;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class Mission {
	private Map<String,Robot> robots = new LinkedHashMap<String,Robot>();
	private Map<String,Computer> computers = new LinkedHashMap<String,Computer>();
	private List<EnvironmentalObject> objects = new ArrayList<EnvironmentalObject>();
	
	private List<Message> messages = new ArrayList<Message>();
	
	public List<Robot> getAllRobots() {
		return new ArrayList<Robot>(robots.values());
	}
	
	public EnvironmentalObject getEnvironmentalObject(int label) {
		EnvironmentalObject eo = objects.get(label);
		if (eo.getLabel() == label)	return eo;
		else return null;
	}
	
	public List<Computer> getAllComputers() {
		return new ArrayList<Computer>(computers.values());
	}
	
	public Robot getRobot(String name) {
		return robots.get(name);
	}
	
	public Computer getComputer(String name) {
		return computers.get(name);
	}
	
	public boolean includesComputer() {
		return (computers.size() > 0); 
	}
	
	public void addRobot(Robot r) {
		robots.put(r.getName(), r);
	}
	
	public void addMessage(Message m) {
		messages.add(m);
	}
	
	//TODO: setters for the other geometric regions as well
	public void addObject(EnvironmentalObject eo) {
		objects.add(eo);
	}
	
	public void addComputer(Computer c) {
		computers.put(c.name, c);
	}

	public List<EnvironmentalObject> getEnvironmentalObjects() {
		return objects;
	}
	
	public List<Message> messagesToComponent(Component c) {
        List<Message> res = messages.stream()
                .filter(msg -> msg.isTo(c))
                .collect(Collectors.toList());
        return res;
	}
	
	public List<Message> messagesFromComponent(Component c) {
        List<Message> res = messages.stream()
                .filter(msg -> msg.isFrom(c))
                .collect(Collectors.toList());
        return res;
	}
	
	public List<Message> messagesToAnyComponent(List<Component> cs) {
        List<Message> res = messages.stream()
                .filter(msg -> cs.contains(msg.getTo()))
                .collect(Collectors.toList());
        return res;
	}
	
	public List<Message> messagesFromAnyComponent(List<Component> cs) {
        List<Message> res = messages.stream()
                .filter(msg -> cs.contains(msg.getFrom()))
                .collect(Collectors.toList());
        return res;
	}
}
