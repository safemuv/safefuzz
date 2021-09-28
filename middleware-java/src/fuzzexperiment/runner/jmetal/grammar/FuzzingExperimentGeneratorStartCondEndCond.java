package fuzzexperiment.runner.jmetal.grammar;

import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartEnd;
import it.units.malelab.jgea.representation.tree.Tree;

import java.util.List;
import atlasdsl.Mission;
import fuzzingengine.*;

public class FuzzingExperimentGeneratorStartCondEndCond extends FuzzingExperimentGeneratorConstraints {
	public FuzzingExperimentGeneratorStartCondEndCond(Mission mission, Grammar<String> grammar) {
		super(mission, grammar);
	}
	
	protected FuzzingTimeSpecification generateFuzzingTimeSpec(VariableSpecification var) throws TreeGenerationFailed {
		// Want two trees here - for start and end conditions
		int n = 2;
		List<Tree<String>> res = grammarGenerator.build(n, rngGenerator);
		Tree<String> startCondTree = res.get(0);
		Tree<String> endCondTree = res.get(1);
		
		if (startCondTree == null) {
			throw new TreeGenerationFailed("startCondTree");
		}
		
		if (endCondTree == null) {
			throw new TreeGenerationFailed("endCondTree");
		}
		
		System.out.print("startCondTree = ");
		startCondTree.prettyPrintLine(System.out);
		System.out.println();
		
		System.out.print("endCondTree = ");
		endCondTree.prettyPrintLine(System.out);
		System.out.println();
		
		FuzzingCondition startCond = new FuzzingCondition(startCondTree);
		FuzzingCondition endCond = new FuzzingCondition(endCondTree);
		FuzzingConditionStartEnd fts = new FuzzingConditionStartEnd(startCond, endCond);
		return fts;
	}

}
