package fuzzingengine.exptgenerator.condition;

import java.util.List;

import atlasdsl.Mission;
import fuzzexperiment.runner.jmetal.grammar.TreeGenerationFailed;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.exptgenerator.*;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartSpec;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import fuzzingengine.operations.FuzzingOperation;

public class FuzzingExperimentFresh extends fuzzingengine.exptgenerator.FuzzingExperimentFresh {

	public FuzzingExperimentFresh(Mission mission) {
		super(mission);
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

			// Set up the timing from the mission constraints - for now, the full time range

			// Modify it to use conditions here
			FuzzingCondition startCond = FuzzingExperimentConditionCreator.generateValidatedCondition();
			double startTime = getStartTime(var.getTimeSpec());
			double endTime = getEndTime(var.getTimeSpec(), startTime);
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
