package fuzzingengine.grammar.conditionelements;

import middleware.core.ATLASCore;

public class FuzzingConditionComparison extends FuzzingConditionElement {
	public enum ComparisonOperation {
		LESS_THAN,
		GREATER_THAN,
		EQUALS,
		NOT_EQUALS,
	}
	
	private FuzzingConditionElement lhs;
	private FuzzingConditionElement rhs;
	private ComparisonOperation op;
	
	public FuzzingConditionComparison(FuzzingConditionElement lhs, FuzzingConditionElement rhs, ComparisonOperation op) {
		this.lhs = lhs;
		this.rhs = rhs;
		this.op = op;
	}
		
	public Object evaluate(ATLASCore core, String vehicle) {
		Object lres = lhs.evaluate(core, vehicle);
		Object rres = rhs.evaluate(core, vehicle);
		
		if ((lres instanceof Double) && (rres instanceof Double)) {
			double lres_n = (double)lres;
			double rres_n = (double)rres;
		
			switch (op) {
				case LESS_THAN: 
					return lres_n < rres_n;
				case GREATER_THAN: 
					return lres_n > rres_n;
				case EQUALS:
					return lres_n == rres_n; 
				case NOT_EQUALS: 
					return lres_n != rres_n;
				default:
					return false;
			}
		} else {
			System.out.println("FuzzingConditionComparison: output not doubles on both sides");
			return false;
		}
	}

	public void validate() throws InvalidFuzzingConditionElement {
		// TODO Auto-generated method stub
		if (lhs instanceof FuzzingConditionVariable && rhs instanceof FuzzingConditionVariable) {
			FuzzingConditionVariable lhsV = (FuzzingConditionVariable)lhs;
			FuzzingConditionVariable rhsV = (FuzzingConditionVariable)rhs;
			if (lhsV.getVariableName().equals(rhsV.getVariableName())) {
				throw new InvalidFuzzingConditionElement("lhs " + lhsV + " in comparison with " + rhsV);
			}
		}
	}
}
