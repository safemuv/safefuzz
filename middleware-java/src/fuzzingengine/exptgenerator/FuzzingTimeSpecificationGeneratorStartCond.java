package fuzzingengine.exptgenerator;

import java.util.Random;

import atlasdsl.Mission;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartSpec;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.exptgenerator.condition.FuzzingExperimentConditionCreator;

public class FuzzingTimeSpecificationGeneratorStartCond extends FuzzingTimeSpecificationGeneratorStartEnd {
	protected Mission mission;
	
	public FuzzingTimeSpecificationGeneratorStartCond(Mission mission, Random rng) {
		super(mission, rng);
		this.mission = mission;
	}

	public FuzzingTimeSpecification gen(VariableSpecification var) {
		double startTime = getStartTime(var.getTimeSpec());
		double endTime = getEndTime(var.getTimeSpec(), startTime);
		FuzzingCondition startCond = FuzzingExperimentConditionCreator.generateValidatedCondition();
		FuzzingConditionStartSpec fts = new FuzzingConditionStartSpec(startCond, endTime);
		return fts;
	}
}
