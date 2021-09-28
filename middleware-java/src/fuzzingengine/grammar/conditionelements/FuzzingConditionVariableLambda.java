package fuzzingengine.grammar.conditionelements;

import middleware.core.ATLASCore;
import middleware.core.ObjectLambda;

public class FuzzingConditionVariableLambda extends FuzzingConditionVariable {
	ObjectLambda fixedValue;
	
	public FuzzingConditionVariableLambda(String name, ObjectLambda fixedValue) {
		super(name);
		this.fixedValue = fixedValue;
	}
	
	public Object evaluate(ATLASCore core, String vehicle) {
		return fixedValue.op(vehicle);
	}
}
