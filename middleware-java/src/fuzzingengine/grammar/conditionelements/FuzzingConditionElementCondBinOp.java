package fuzzingengine.grammar.conditionelements;

import middleware.core.ATLASCore;

public class FuzzingConditionElementCondBinOp extends FuzzingConditionElementCondComposite {
	public enum FuzzingConditionBinLogicOp {
		AND,
		OR
	}
	
	private FuzzingConditionElement lhs;
	private FuzzingConditionElement rhs;
	private FuzzingConditionBinLogicOp op;
	
	public FuzzingConditionElementCondBinOp(FuzzingConditionElement lhs, FuzzingConditionElement rhs, FuzzingConditionBinLogicOp op) {
		this.lhs = lhs;
		this.rhs = rhs;
		this.op = op;
	}
		
	public Object evaluate(ATLASCore core, String vehicle) {
		Object lres = lhs.evaluate(core, vehicle);
		Object rres = rhs.evaluate(core, vehicle);
		
		if ((lres instanceof Boolean) && (rres instanceof Boolean)) {
			Boolean lres_n = (boolean)lres;
			Boolean rres_n = (boolean)rres;
		
			switch (op) {
				case AND: 
					return lres_n && rres_n;
				case OR: 
					return lres_n || rres_n;
				default:
					return false;
			}
		} else {
			System.out.println("FuzzingConditionComparison: output not bools on both sides");
			return false;
		}
	}
}
