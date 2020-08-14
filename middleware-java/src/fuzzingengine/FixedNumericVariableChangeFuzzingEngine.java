package fuzzingengine;

public class FixedNumericVariableChangeFuzzingEngine extends NumericVariableChangeFuzzingEngine {
	private final double defaultFixedChange = 0.0;
	private String fixedChange = Double.toString(defaultFixedChange);
	
	public FixedNumericVariableChangeFuzzingEngine() {
		
	}
	
	public FixedNumericVariableChangeFuzzingEngine(double fixedChange) {
		this.fixedChange = Double.toString(fixedChange);
	}
	
	public String getReplacement(String inValue) {
		return fixedChange;
	}
}
