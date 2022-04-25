package fuzzexperiment.runner.jmetal.grammar.variables;

import middleware.core.ATLASCore;

public abstract class VariableTemplate {
	public abstract double getValue(String robotName, ATLASCore core);
}
