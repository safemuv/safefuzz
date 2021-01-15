package fuzzingengine.operations;

public class CreationFailed extends Exception {
	private static final long serialVersionUID = 1L;
	
	String info;
	public CreationFailed(String info) {
		this.info = info;
	}
}
