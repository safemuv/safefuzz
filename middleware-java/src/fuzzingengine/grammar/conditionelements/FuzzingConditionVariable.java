package fuzzingengine.grammar.conditionelements;

import middleware.core.ATLASCore;

public class FuzzingConditionVariable extends FuzzingConditionElement {
	String name;
	
	public FuzzingConditionVariable(String name) {
		this.name = name;
	}

	public Object evaluate(ATLASCore core, String vehicle) {
		Object val = core.getVariable(name, vehicle);
		return val;
	}
	
	public String getVariableName() {
		return name;
	}

	public void validate() throws InvalidFuzzingConditionElement {
		
	}
}
