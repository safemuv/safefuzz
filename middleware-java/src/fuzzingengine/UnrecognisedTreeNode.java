package fuzzingengine;

public class UnrecognisedTreeNode extends Exception {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private String compareStr;
	
	public UnrecognisedTreeNode(String str) {
		this.compareStr = str;
	}
}
