package fuzzingengine.exptgenerator.condition;

import java.io.IOException;
import java.util.List;
import java.util.Random;

import atlasdsl.Mission;
import fuzzexperiment.runner.jmetal.grammar.Grammar;
import fuzzexperiment.runner.jmetal.grammar.GrowGrammarTreeFactory;
import fuzzexperiment.runner.jmetal.grammar.TreeGenerationFailed;
import fuzzingengine.InvalidCondition;
import fuzzingengine.UnrecognisedBinOp;
import fuzzingengine.UnrecognisedComparison;
import fuzzingengine.UnrecognisedTreeNode;
import fuzzingengine.UnrecognisedUnOp;
import fuzzingengine.exptgenerator.FuzzingExperimentGenerator;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.TestEvaluateConditions;
import it.units.malelab.jgea.representation.tree.Tree;

public class FuzzingExperimentConditionCreator  {
	public FuzzingExperimentConditionCreator(Mission mission) {

	}

	Grammar<String> grammar;
	public static GrowGrammarTreeFactory<String> grammarGenerator;
	public static Random rngGenerator;
	public static TestEvaluateConditions testCond = new TestEvaluateConditions();
	
	protected static final boolean VALIDATE_CONDITIONS = true;
	private static final boolean ANALYSE_CONDITION = true;
	
	public static FuzzingCondition generateValidatedCondition() throws TreeGenerationFailed {
		int numTrees = 1;
		while (true) {
			
			// TODO: temporary height of 4
			int randomHeight = 4;
			
			List<Tree<String>> res = grammarGenerator.build(numTrees, rngGenerator, randomHeight);
			Tree<String> t = res.get(0);
			if (t == null) {
				throw new TreeGenerationFailed("t in generateValidatedCondition");
			}
			
			FuzzingCondition c = new FuzzingCondition(t);
			// If not validating, return the first generated condition
			if (!VALIDATE_CONDITIONS) {
				return c;
			} else {
				try {
					c.doConversion();
					c.validateCondition();
					
					if (ANALYSE_CONDITION) {
						// analyse condition true/false structure 
						try {
							System.out.println(c.toString());
							testCond.testEvaluateCondition(t, "uav_1", "evalcondition-uav_1-" + c.toString() + ".res");
							testCond.testEvaluateCondition(t, "uav_2", "evalcondition-uav_2-" + c.toString() + ".res");
						} catch (IOException e) {
							e.printStackTrace();
						}
					}
					
					System.out.println("Validated condition - " + c.csvPrint());
					return c;
				} catch (UnrecognisedComparison | UnrecognisedTreeNode | UnrecognisedUnOp | UnrecognisedBinOp e) {
					e.printStackTrace();
				} catch (InvalidCondition e) {
					System.out.println("Invalid condition... retrying");
					//e.printStackTrace();
				}	
			}
		}
	}
}
