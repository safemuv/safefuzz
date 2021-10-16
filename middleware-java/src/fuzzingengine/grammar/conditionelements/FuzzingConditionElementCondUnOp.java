package fuzzingengine.grammar.conditionelements;

import middleware.core.ATLASCore;

public class FuzzingConditionElementCondUnOp extends FuzzingConditionElementCondComposite {
	public enum FuzzingConditionUnLogicOp {
		NOT
	}
	
	private FuzzingConditionElement elt;
	private FuzzingConditionUnLogicOp op;
	
	public FuzzingConditionElementCondUnOp(FuzzingConditionElement elt, FuzzingConditionUnLogicOp op) {
		this.elt = elt;
		this.op = op;
	}
		
	public Object evaluate(ATLASCore core, String vehicle) {
		Object lres = elt.evaluate(core, vehicle);
		
		if ((lres instanceof Boolean)) {
			Boolean lres_n = (boolean)lres;
		
			switch (op) {
				case NOT: 
					return !lres_n;
				default:
					return false;
			}
		} else {
			System.out.println("FuzzingConditionComparison: output not doubles on both sides");
			return false;
		}
	}
}
