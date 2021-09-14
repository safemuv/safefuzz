package fuzzingengine.conditionelements;

import java.util.ArrayList;
import java.util.List;

import middleware.core.ATLASCore;

public class FuzzingConditionElementCond extends FuzzingConditionElement {
	List<FuzzingConditionElement> conds = new ArrayList<FuzzingConditionElement>();
	
	public FuzzingConditionElementCond() {
		
	}
	
	public Object evaluate(ATLASCore core, String vehicle) {
		boolean output = true;
		
		for (FuzzingConditionElement cond : conds) {
			Object res = cond.evaluate(core, vehicle);
			if (res instanceof Boolean) {
				output = output & (boolean)res;
			} else {
				output = false;
			}
		}
		return output;
	}

	public void addSubcondition(FuzzingConditionElement cond) {
		conds.add(cond);
	}
}
