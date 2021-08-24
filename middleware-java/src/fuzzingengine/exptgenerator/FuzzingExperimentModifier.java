package fuzzingengine.exptgenerator;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;

import atlasdsl.Mission;
import fuzzexperiment.runner.metrics.Metric;
import fuzzingengine.FuzzingFixedTimeSpecification;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.TimeSpec;
import fuzzingengine.operationparamsinfo.OperationParameterSet;

public class FuzzingExperimentModifier extends FuzzingExperimentFresh {

	private final double PROB_OF_TEMPORAL_MUTATION = 1.0 / 3.0;
	private final double PROB_OF_PARAM_CHANGE = 1.0 / 3.0;

	Random rng;
	private Mission mission;

	public enum ChangeOp {
		SHIFT_TIME, 
		CHANGE_PARAM,
		CHANGE_PARTICIPANTS
	}

	public FuzzingExperimentModifier(Mission mission) {
		super(mission);
		rng = new Random();
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
			krec = selectRandomElementFrom(output, "output in generateExperimentBasedUpon");
			
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

			outputAsCSV(newFile, output);

		} catch (ListHasNoElement e) {
			System.out.println("generateExperimentBasedUpon - empty original experiment");
		} catch (IOException e) {
			e.printStackTrace();
		} catch (OperationLoadFailed e) {
			e.printStackTrace();
		}
		return output;
	}
	
	private void adjustTime(FuzzingSelectionRecord m) {
		double startTime;
		double endTime;
		
		FuzzingSimMapping fsm = fuzzEngine.getSimMapping();
		if (m instanceof FuzzingKeySelectionRecord) {
			FuzzingKeySelectionRecord km = (FuzzingKeySelectionRecord)m;
			VariableSpecification vs = fsm.getRecords().get(km.getKey());
			Optional<TimeSpec> ts = vs.getTimeSpec();
			
			FuzzingTimeSpecification zts = km.getTimeSpec();
			if (zts instanceof FuzzingFixedTimeSpecification) {
				FuzzingFixedTimeSpecification fts = (FuzzingFixedTimeSpecification)zts;
				startTime = getStartTime(ts);
				endTime = getEndTime(ts, startTime);
				fts.setStartTime(startTime);
				fts.setEndTime(endTime);
			}
			
			// TODO: Set the mutations here for the other types

		} else {
			startTime = getStartTime(Optional.empty());
			endTime = getEndTime(Optional.empty(), startTime);
		}
	}
	
	private void newParameters(FuzzingSelectionRecord m) throws ListHasNoElement, OperationLoadFailed {
		FuzzingSimMapping fsm = fuzzEngine.getSimMapping();
		for (Map.Entry<String, VariableSpecification> vs : fsm.getRecords().entrySet()) {
			VariableSpecification var = vs.getValue();
			
			if (m instanceof FuzzingKeySelectionRecord) {
				FuzzingKeySelectionRecord kr = (FuzzingKeySelectionRecord)m;
				// Need to check the variable spec matches the chosen key!
				if (var.getVariable().equals(kr.getKey())) {
					// Select an operation parameter set to use
					List<OpParamSetType> opsetTypes = var.getOperationParamSets();
					OpParamSetType opsetType = selectRandomElementFrom(opsetTypes, "opsetType in newParameters");
					OperationParameterSet opset = opsetType.getOpset();
					List<Object> specificOpParams = opset.generateSpecific();
					kr.setParams(specificOpParams);
				}
			}
		}
	}
	
	private void newParticipants(FuzzingSelectionRecord m) throws ListHasNoElement, OperationLoadFailed {
		List<String> newParticipants = getRandomParticipantsFromMission();
		((FuzzingKeySelectionRecord)m).setParticipants(newParticipants);
	}
}
