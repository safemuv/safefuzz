package fuzzingengine.operations;

public abstract class ValueFuzzingOperation extends FuzzingOperation {
	public abstract String fuzzTransformString(String input);

	public boolean isEventBased() {
		return false;
	}
	
	public boolean isValueBased() {
		return true;
	}
}
