package fuzzexperiment.runner.jmetal.grammar;

import java.util.List;
import java.util.Random;

import atlasdsl.Mission;
import fuzzingengine.FuzzingCondition;
import fuzzingengine.FuzzingConditionStartSpec;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.conditionelements.FuzzingConditionElement;
import fuzzingengine.exptgenerator.FuzzingExperimentGenerator;
import fuzzingengine.exptgenerator.ListHasNoElement;
import fuzzingengine.exptgenerator.OperationLoadFailed;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import fuzzingengine.operations.FuzzingOperation;
import it.units.malelab.jgea.core.Factory;
import it.units.malelab.jgea.representation.tree.Tree;

public class FuzzingExperimentGeneratorConstraints<T> extends FuzzingExperimentGenerator {
	
	int MIN_TREE_HEIGHT = 1;
	int MAX_TREE_HEIGHT = 3;
	
	// TODO: fix type for grammar elements
	Grammar<FuzzingConditionElement> grammar;
	Factory<Tree<FuzzingConditionElement>> grammarGenerator;
	Random rngGenerator;

	public FuzzingExperimentGeneratorConstraints(Mission mission, Grammar<FuzzingConditionElement> grammar) {
		super(mission);
		this.grammar = grammar;
		//this.grammarGenerator = new GrammarRampedHalfAndHalf(MIN_TREE_HEIGHT, MAX_TREE_HEIGHT, grammar);
		rngGenerator = new Random();
	}

	protected FuzzingKeySelectionRecord generateVariableEntry(VariableSpecification var)
			throws OperationLoadFailed, ListHasNoElement {
		// Select an operation parameter set to use
		List<OpParamSetType> opsetTypes = var.getOperationParamSets();
		if (opsetTypes.size() == 0) {
			throw new ListHasNoElement("opsetType in generateVariableEntry");
		} else {
			OpParamSetType opsetType = selectRandomElementFrom(opsetTypes, "opsetType in generateVariableEntry");
			OperationParameterSet opset = opsetType.getOpset();
			String subSpec = opsetType.getSubSpec();

			List<Object> specificOpParams = opset.generateSpecific();

			// Set up the grammar generation - for now, the condition for start, and then the timing for end
			// TODO: value of n?
			int n = MAX_TREE_HEIGHT;
			List<Tree<FuzzingConditionElement>> specTree = grammarGenerator.build(n, rngGenerator);

			double startTime = getStartTime(var.getTimeSpec());
			double endTime = getEndTime(var.getTimeSpec(), startTime);
			FuzzingCondition startCond = new FuzzingCondition(specTree);
			FuzzingConditionStartSpec fts = new FuzzingConditionStartSpec(startCond, endTime);

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
