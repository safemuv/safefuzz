package fuzzingengine.grammar;

import java.util.Map;
import fuzzingengine.grammar.conditionelements.FuzzingConditionElement;
import fuzzingengine.grammar.conditionelements.FuzzingConditionVariableLambda;
import middleware.core.ObjectLambda;

public class GrammarConvertorFixedVarFunctions extends GrammarConvertor {
	Map<String,ObjectLambda> fixedVariableValues;
	
	class MissingValueInMap extends RuntimeException {
		private static final long serialVersionUID = 1L;
		private String varName;
		
		public MissingValueInMap(String varName) {
			this.varName = varName;
		}
	}
	
	public GrammarConvertorFixedVarFunctions(Map<String,ObjectLambda> fixedVariableValues) {
		super();
		this.fixedVariableValues = fixedVariableValues;
	}
	
	protected FuzzingConditionElement createVariableEntry(String varName) throws MissingValueInMap {
		if (fixedVariableValues.containsKey(varName)) {
			ObjectLambda vl = fixedVariableValues.get(varName);
			return new FuzzingConditionVariableLambda(varName, vl);
		} else {
			System.out.println("MissingValueInMap = " + varName);
			throw new MissingValueInMap(varName);
		}
	}
	
	public void updateFunctions(Map<String,ObjectLambda> fmap) {
		this.fixedVariableValues = fmap;
	}
}
