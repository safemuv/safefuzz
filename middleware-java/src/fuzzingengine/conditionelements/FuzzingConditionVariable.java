package fuzzingengine.conditionelements;

import middleware.core.ATLASCore;

public class FuzzingConditionVariable extends FuzzingConditionElement {
	String name;
	
	public FuzzingConditionVariable(String name) {
		this.name = name;
	}

	public Object evaluate(ATLASCore core) {
		// TODO: Need to get the named simulator variable from a cache and inject it here 
		Object val = core.getSimVariable(name);
		return val;
	}
}
