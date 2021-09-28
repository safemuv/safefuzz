package fuzzingengine.grammar;

import java.util.Map;
import fuzzingengine.grammar.conditionelements.FuzzingConditionElement;
import fuzzingengine.grammar.conditionelements.FuzzingConditionVariableLambda;
import middleware.core.ObjectLambda;

public class GrammarConvertorFixedVarFunctions extends GrammarConvertor {
	Map<String,ObjectLambda> fixedVariableValues;
	
	public GrammarConvertorFixedVarFunctions(Map<String,ObjectLambda> fixedVariableValues) {
		super();
		this.fixedVariableValues = fixedVariableValues;
	}
	
	protected FuzzingConditionElement createVariableEntry(String varName) {
		ObjectLambda vl = fixedVariableValues.get(varName);
		return new FuzzingConditionVariableLambda(varName, vl);
	}
	
	public void updateFunctions(Map<String,ObjectLambda> fmap) {
		this.fixedVariableValues = fmap;
	}
}
