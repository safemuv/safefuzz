package fuzzingengine;

public class MissingRobot extends Exception {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private String name;
	
	public MissingRobot(String name) {
		this.name = name;
	}
	
	public String getName() {
		return name;
	}
}
