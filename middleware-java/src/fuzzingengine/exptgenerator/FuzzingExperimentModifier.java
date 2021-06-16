package fuzzingengine.exptgenerator;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;

import atlasdsl.Mission;
import fuzzexperiment.runner.metrics.Metric;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import fuzzingengine.operations.FuzzingOperation;

public class FuzzingExperimentModifier extends FuzzingExperimentGenerator {

	private final double PROB_OF_TEMPORAL_MUTATION = 1.0 / 3;
	private final double PROB_OF_PARAM_CHANGE = 1.0 / 3;

	Random rng;
	private Mission mission;

	public enum ChangeOp {
		SHIFT_TIME, 
		CHANGE_PARAM,
		CHANGE_PARTICIPANTS
	}

	public FuzzingExperimentModifier(Mission mission) {
		super(mission);
	}
	
	public ChangeOp selectRandomOperation() {
		double v = rng.nextDouble();
		if (v < PROB_OF_TEMPORAL_MUTATION) {
			return ChangeOp.SHIFT_TIME;
		} else { 
			if (v < PROB_OF_TEMPORAL_MUTATION + PROB_OF_PARAM_CHANGE) {
				return ChangeOp.CHANGE_PARAM;
			} else {
				return ChangeOp.CHANGE_PARTICIPANTS;
			}
		}
	}
	
	public List<FuzzingSelectionRecord> duplicateExperimentRecords(List<FuzzingSelectionRecord> input) {
		List<FuzzingSelectionRecord> output = new ArrayList<FuzzingSelectionRecord>();
		for (FuzzingSelectionRecord r : input) {
			output.add(r.dup());
		}
		
		return output;
	}

	public List<FuzzingSelectionRecord> generateExperimentBasedUpon(String newFile, List<FuzzingSelectionRecord> basis,
			Map<Metric, Double> res) {
		// Modify the timings or parameters of one of the values
		FuzzingSelectionRecord krec;
		List<FuzzingSelectionRecord> output = duplicateExperimentRecords(basis);
		
		try {
			krec = selectRandomElementFrom(output);
			ChangeOp operation = selectRandomOperation();
			System.out.println("Mutation operation selected = " + operation);
			
			if (operation == ChangeOp.SHIFT_TIME) {
				adjustTime(krec);
			}

			if (operation == ChangeOp.CHANGE_PARAM) {
				newParameters(krec);
			}
			
			if (operation == ChangeOp.CHANGE_PARTICIPANTS) {
				newParticipants(krec);
			}

			outputAsCSV(newFile, basis);

		} catch (ListHasNoElement e) {
			System.out.println("generateExperimentBasedUpon - empty original experiment");
		} catch (IOException e) {
			e.printStackTrace();
		} catch (OperationLoadFailed e) {
			e.printStackTrace();
		}
		return basis;
	}

	private void adjustTime(FuzzingSelectionRecord m) {
		double startTime = getStartTime(Optional.empty());
		double endTime = getStartTime(Optional.empty());
		m.setStartTime(startTime);
		m.setEndTime(endTime);
	}
	
	private void newParameters(FuzzingSelectionRecord m) throws ListHasNoElement, OperationLoadFailed {
		FuzzingSimMapping fsm = fuzzEngine.getSimMapping();
		for (Map.Entry<String, VariableSpecification> vs : fsm.getRecords().entrySet()) {
		// Select an operation parameter set to use
			VariableSpecification var = vs.getValue();
			List<OpParamSetType> opsetTypes = var.getOperationParamSets();
			if (opsetTypes.size() == 0) {
				throw new ListHasNoElement();
			} else {
				OpParamSetType opsetType = selectRandomElementFrom(opsetTypes);
				OperationParameterSet opset = opsetType.getOpset();
				String subSpec = opsetType.getSubSpec();
				List<Object> specificOpParams = opset.generateSpecific();
				String paramStr = paramsAsString(specificOpParams);
				FuzzingOperation op = loadOperationFromParams(opset.getOperationClassName(), paramStr);
				((FuzzingKeySelectionRecord)m).setParams(specificOpParams);
			}
		}
	}
	
	private void newParticipants(FuzzingSelectionRecord m) throws ListHasNoElement, OperationLoadFailed {
		List<String> newParticipants = getRandomParticipantsFromMission();
		((FuzzingKeySelectionRecord)m).setParticipants(newParticipants);
	}
	
	
}