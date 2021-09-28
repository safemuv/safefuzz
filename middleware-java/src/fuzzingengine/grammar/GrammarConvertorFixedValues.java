package fuzzingengine.grammar;

import java.util.Map;
import fuzzingengine.grammar.conditionelements.FuzzingConditionElement;
import fuzzingengine.grammar.conditionelements.FuzzingConditionVariableFakeConstant;

public class GrammarConvertorFixedValues extends GrammarConvertor {
	Map<String,Object> fixedVariableValues;
	
	public GrammarConvertorFixedValues(Map<String,Object> fixedVariableValues) {
		super();
		this.fixedVariableValues = fixedVariableValues;
	}
	
	protected FuzzingConditionElement createVariableEntry(String varName) {
		Object v = fixedVariableValues.get(varName);
		return new FuzzingConditionVariableFakeConstant(varName, v);
	}
}
