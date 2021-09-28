package fuzzingengine.grammar.conditionelements;

import middleware.core.ATLASCore;

public class FuzzingConditionVariable extends FuzzingConditionElement {
	String name;
	
	public FuzzingConditionVariable(String name) {
		this.name = name;
	}

	public Object evaluate(ATLASCore core, String vehicle) {
		// TODO: Need to get the named simulator variable from a cache and inject it here 
		Object val = core.getVariable(name, vehicle);
		return val;
	}
}
