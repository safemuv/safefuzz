package fuzzexperiment.runner.jmetal.grammar;

public class TreeGenerationFailed extends Exception {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private String reason;
	
	public TreeGenerationFailed(String reason) {
		this.reason = reason;
	}

}
