package fuzzexperiment.runner.jmetal.grammar;

import java.io.IOException;
import java.util.List;
import java.util.Random;

import atlasdsl.Mission;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.InvalidCondition;
import fuzzingengine.UnrecognisedBinOp;
import fuzzingengine.UnrecognisedComparison;
import fuzzingengine.UnrecognisedTreeNode;
import fuzzingengine.UnrecognisedUnOp;
import fuzzingengine.exptgenerator.FuzzingExperimentGenerator;
import fuzzingengine.exptgenerator.ListHasNoElement;
import fuzzingengine.exptgenerator.OperationLoadFailed;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartSpec;
import fuzzingengine.grammar.TestEvaluateConditions;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import fuzzingengine.operations.FuzzingOperation;
import it.units.malelab.jgea.core.Factory;
import it.units.malelab.jgea.representation.tree.Tree;

public abstract class FuzzingExperimentGeneratorConstraints extends FuzzingExperimentGenerator {
	int MIN_TREE_HEIGHT = 4;
	int MAX_TREE_HEIGHT = 6;
	protected static final boolean VALIDATE_CONDITIONS = true;
	private static final boolean ANALYSE_CONDITION = true;
	
	private static int fileNum = 0;
	
	Grammar<String> grammar;
	GrowGrammarTreeFactory<String> grammarGenerator;
	Random rngGenerator;
	TestEvaluateConditions testCond = new TestEvaluateConditions();

	public FuzzingExperimentGeneratorConstraints(Mission mission, Grammar<String> grammar) {
		super(mission);
		this.grammar = grammar;
		this.grammarGenerator = new GrowGrammarTreeFactory<String>(MAX_TREE_HEIGHT, grammar);
		rngGenerator = new Random();
	}
	
	protected abstract FuzzingTimeSpecification generateFuzzingTimeSpec(VariableSpecification var) throws TreeGenerationFailed;

	protected FuzzingCondition generateValidatedCondition() throws TreeGenerationFailed {
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
	
	protected FuzzingKeySelectionRecord generateVariableEntry(VariableSpecification var)
			throws OperationLoadFailed, ListHasNoElement, TreeGenerationFailed {
		// Select an operation parameter set to use
		List<OpParamSetType> opsetTypes = var.getOperationParamSets();
		if (opsetTypes.size() == 0) {
			throw new ListHasNoElement("opsetType in generateVariableEntry");
		} else {
			OpParamSetType opsetType = selectRandomElementFrom(opsetTypes, "opsetType in generateVariableEntry");
			OperationParameterSet opset = opsetType.getOpset();
			String subSpec = opsetType.getSubSpec();

			List<Object> specificOpParams = opset.generateSpecific();

			FuzzingTimeSpecification fts = generateFuzzingTimeSpec(var);



			// Get a random list of participants from the mission
			List<String> participants = getRandomParticipantsFromMission();

			// TODO: merge the specificOpParams with pipes
			String paramStr = paramsAsString(specificOpParams);
			FuzzingOperation op = loadOperationFromParams(opset.getOperationClassName(), paramStr);

			FuzzingKeySelectionRecord ksr = new FuzzingKeySelectionRecord(var.getVariable(),
					var.getReflectionName_opt(), var.getComponent(), var.getRegexp(), subSpec, op, participants, fts);
			ksr.setParams(specificOpParams);
			return ksr;
		}
	}
}
