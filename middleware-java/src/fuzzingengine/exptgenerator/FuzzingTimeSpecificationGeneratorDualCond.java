package fuzzingengine.exptgenerator;

import java.util.Random;

import atlasdsl.Mission;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartEnd;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.exptgenerator.condition.FuzzingExperimentConditionCreator;

public class FuzzingTimeSpecificationGeneratorDualCond extends FuzzingTimeSpecificationGeneratorStartEnd {
	protected Mission mission;
	
	public FuzzingTimeSpecificationGeneratorDualCond(Mission mission, Random rng) {
		super(mission, rng);
		this.mission = mission;
	}

	public FuzzingTimeSpecification gen(VariableSpecification var) {
		FuzzingCondition startCond = FuzzingExperimentConditionCreator.generateValidatedCondition();
		FuzzingCondition endCond = FuzzingExperimentConditionCreator.generateValidatedCondition();
		FuzzingTimeSpecification fts = new FuzzingConditionStartEnd(startCond, endCond);
		return fts;
	}
}
