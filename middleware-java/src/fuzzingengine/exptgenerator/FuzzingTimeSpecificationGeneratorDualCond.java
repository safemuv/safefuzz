package fuzzingengine.exptgenerator;

import java.util.Random;

import atlasdsl.Mission;
import fuzzexperiment.runner.jmetal.grammar.GrammarRampedHalfAndHalf;
import fuzzexperiment.runner.jmetal.grammar.GrowGrammarTreeFactory;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartEnd;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.exptgenerator.condition.FuzzingExperimentConditionCreator;

public class FuzzingTimeSpecificationGeneratorDualCond extends FuzzingTimeSpecificationGeneratorStartEnd {
	protected Mission mission;
	private GrowGrammarTreeFactory<String> gen;
	
	public FuzzingTimeSpecificationGeneratorDualCond(Mission mission, Random rng, GrowGrammarTreeFactory<String> gen) {
		super(mission, rng);
		this.mission = mission;
		this.gen = gen;
	}

	public FuzzingTimeSpecification gen(VariableSpecification var) {
		FuzzingCondition startCond = FuzzingExperimentConditionCreator.generateValidatedCondition(rng, gen);
		FuzzingCondition endCond = FuzzingExperimentConditionCreator.generateValidatedCondition(rng, gen);
		FuzzingTimeSpecification fts = new FuzzingConditionStartEnd(startCond, endCond);
		return fts;
	}
}
