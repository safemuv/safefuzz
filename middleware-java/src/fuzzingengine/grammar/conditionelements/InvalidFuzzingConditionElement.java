package fuzzingengine.grammar.conditionelements;

public class InvalidFuzzingConditionElement extends Exception {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	String reason;
	
	public InvalidFuzzingConditionElement(String reason) {
		this.reason = reason;
	}
}
