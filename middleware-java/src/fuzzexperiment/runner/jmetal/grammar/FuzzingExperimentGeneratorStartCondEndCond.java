package fuzzexperiment.runner.jmetal.grammar;

import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartEnd;

import atlasdsl.Mission;
import fuzzingengine.*;

public class FuzzingExperimentGeneratorStartCondEndCond extends FuzzingExperimentGeneratorConstraints {


	public FuzzingExperimentGeneratorStartCondEndCond(Mission mission, Grammar<String> grammar) {
		super(mission, grammar);
	}
	
	protected FuzzingTimeSpecification generateFuzzingTimeSpec(VariableSpecification var) throws TreeGenerationFailed {
		// Want two trees here - for start and end conditions
		FuzzingCondition startCond = generateValidatedCondition();
		FuzzingCondition endCond = generateValidatedCondition();
		FuzzingConditionStartEnd fts = new FuzzingConditionStartEnd(startCond, endCond);
		return fts;
	}
}
