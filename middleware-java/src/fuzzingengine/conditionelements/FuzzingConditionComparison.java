package fuzzingengine.conditionelements;

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
		
	public Object evaluate(ATLASCore core) {
		Object lres = lhs.evaluate(core);
		Object rres = rhs.evaluate(core);
		
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
}
