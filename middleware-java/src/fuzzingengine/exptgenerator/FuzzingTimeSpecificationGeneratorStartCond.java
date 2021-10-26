package fuzzingengine.exptgenerator;

import java.util.Random;

import atlasdsl.Mission;
import fuzzexperiment.runner.jmetal.grammar.GrowGrammarTreeFactory;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartSpec;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.exptgenerator.condition.FuzzingExperimentConditionCreator;

public class FuzzingTimeSpecificationGeneratorStartCond extends FuzzingTimeSpecificationGeneratorStartEnd {
	protected Mission mission;
	private GrowGrammarTreeFactory<String> gen;
	
	public FuzzingTimeSpecificationGeneratorStartCond(Mission mission, Random rng, GrowGrammarTreeFactory<String> gen) {
		super(mission, rng);
		this.mission = mission;
		this.gen = gen;
	}

	public FuzzingTimeSpecification gen(VariableSpecification var) {
		double startTime = getStartTime(var.getTimeSpec());
		double endTime = getEndTime(var.getTimeSpec(), startTime);
		FuzzingCondition startCond = FuzzingExperimentConditionCreator.generateValidatedCondition(rng, gen);
		FuzzingConditionStartSpec fts = new FuzzingConditionStartSpec(startCond, endTime);
		return fts;
	}
}
