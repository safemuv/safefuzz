package fuzzingengine.conditionelements;

import java.util.ArrayList;
import java.util.List;

import middleware.core.ATLASCore;

public class FuzzingConditionElementCond extends FuzzingConditionElement {
	List<FuzzingConditionElement> conds = new ArrayList<FuzzingConditionElement>();
	
	public FuzzingConditionElementCond() {
		
	}
	
	public Object evaluate(ATLASCore core) {
		if (conds.size() > 0) {
			return conds.get(0).evaluate(core);
		} else {
			return false;
		}
	}

	public void addSubcondition(FuzzingConditionElement cond) {
		conds.add(cond);
	}
}
