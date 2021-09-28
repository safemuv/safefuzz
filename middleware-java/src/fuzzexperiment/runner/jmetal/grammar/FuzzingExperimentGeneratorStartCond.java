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
		// n is one, only want a single tree
		int n = 1;
		List<Tree<String>> res = grammarGenerator.build(n, rngGenerator);
		Tree<String> startTree = res.get(0);
		
		if (startTree == null) {
			throw new TreeGenerationFailed("startTree");
		}
		
		System.out.print("specTree = ");
		startTree.prettyPrintLine(System.out);
		System.out.println();
		
		double startTime = getStartTime(var.getTimeSpec());
		double endTime = getEndTime(var.getTimeSpec(), startTime);
		FuzzingCondition startCond = new FuzzingCondition(startTree);
		FuzzingConditionStartSpec fts = new FuzzingConditionStartSpec(startCond, endTime);
		return fts;
	}

}
