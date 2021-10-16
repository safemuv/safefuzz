package fuzzingengine.grammar.conditionelements;

import middleware.core.ATLASCore;

public class FuzzingConditionVariableFakeConstant extends FuzzingConditionVariable {
	Object fixedValue;
	
	public FuzzingConditionVariableFakeConstant(String name, Object fixedValue) {
		super(name);
		this.fixedValue = fixedValue;
	}
	
	public Object evaluate(ATLASCore core, String vehicle) {
		return fixedValue;
	}
}
