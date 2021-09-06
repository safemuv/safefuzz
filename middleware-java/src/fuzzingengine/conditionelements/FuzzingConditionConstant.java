package fuzzingengine.conditionelements;

import middleware.core.ATLASCore;

public class FuzzingConditionConstant extends FuzzingConditionElement {
	Object constant;
	
	public FuzzingConditionConstant(Object constant) {
		this.constant = constant;
	}

	public Object evaluate(ATLASCore core) {
		// TODO: Need to get the named simulator variable from a cache and inject it here 
		return constant;
	}
}
