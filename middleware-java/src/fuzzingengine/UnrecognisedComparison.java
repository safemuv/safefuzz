package fuzzingengine;

public class UnrecognisedComparison extends Exception {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private String compareStr;
	
	public UnrecognisedComparison(String str) {
		this.compareStr = str;
	}
}
