package fuzzexperiment.runner.jmetal.grammar;

import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartSpec;
import it.units.malelab.jgea.representation.tree.Tree;

import java.util.List;
import java.util.Optional;

import atlasdsl.Mission;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.TimeSpec;

public class FuzzingExperimentGeneratorStartCond extends FuzzingExperimentGeneratorConstraints {
	public FuzzingExperimentGeneratorStartCond(Mission mission, Grammar<String> grammar) {
		super(mission, grammar);
	}
	
	protected FuzzingTimeSpecification generateFuzzingTimeSpec(VariableSpecification var) throws TreeGenerationFailed {
		double startTime = getStartTime(var.getTimeSpec());
		double endTime = getEndTime(var.getTimeSpec(), startTime);
		FuzzingCondition startCond = generateValidatedCondition();
		FuzzingConditionStartSpec fts = new FuzzingConditionStartSpec(startCond, endTime);
		return fts;
	}
}
