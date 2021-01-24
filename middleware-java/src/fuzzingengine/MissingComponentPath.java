package fuzzingengine;

public class MissingComponentPath extends Exception {
	private static final long serialVersionUID = 1L;
	private String component;
	public MissingComponentPath(String component) {
		this.component = component;
	}
}
