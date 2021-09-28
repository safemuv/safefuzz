package fuzzingengine.grammar.conditionelements;

import middleware.core.ATLASCore;

public abstract class FuzzingConditionElement {
	public abstract Object evaluate(ATLASCore core, String vehicle);
}
