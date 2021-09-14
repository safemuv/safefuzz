package fuzzingengine.conditionelements;

import middleware.core.ATLASCore;

public class FuzzingConditionConstant extends FuzzingConditionElement {
	Object constant;
	
	public FuzzingConditionConstant(Object constant) {
		if (constant instanceof Integer) {
			this.constant = Double.valueOf((Integer)constant);
		} else {
			this.constant = constant;
		}
	}

	public Object evaluate(ATLASCore core, String vehicle) {
		return constant;
	}
}
