package fuzzingengine.exptgenerator;

import java.util.List;

import atlasdsl.Mission;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import fuzzingengine.operations.FuzzingOperation;

public class FuzzingExperimentFresh extends FuzzingExperimentGenerator {

	public FuzzingExperimentFresh(Mission mission) {
		super(mission);
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

			// Set up the timing from the mission constraints - for now, the full time range

			double startTime = getStartTime(var.getTimeSpec());
			double endTime = getEndTime(var.getTimeSpec(), startTime);

			// Get a random list of participants from the mission
			List<String> participants = getRandomParticipantsFromMission();

			// TODO: merge the specificOpParams with pipes
			String paramStr = paramsAsString(specificOpParams);
			FuzzingOperation op = loadOperationFromParams(opset.getOperationClassName(), paramStr);

			FuzzingKeySelectionRecord ksr = new FuzzingKeySelectionRecord(var.getVariable(),
					var.getReflectionName_opt(), var.getComponent(), var.getRegexp(), subSpec, op, participants,
					startTime, endTime);
			ksr.setParams(specificOpParams);
			return ksr;
		}
	}	
}
