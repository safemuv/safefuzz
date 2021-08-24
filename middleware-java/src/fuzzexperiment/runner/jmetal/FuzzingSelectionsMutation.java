package fuzzexperiment.runner.jmetal;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.uma.jmetal.operator.mutation.MutationOperator;

import atlasdsl.Mission;
import atlasdsl.Robot;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.exptgenerator.ListHasNoElement;
import fuzzingengine.exptgenerator.OperationLoadFailed;
import fuzzingengine.exptgenerator.FuzzingExperimentModifier.ChangeOp;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import exptrunner.operations.*;
import fuzzexperiment.runner.metrics.Metric;

// TODO: check mutation logic against the changes that I made with Simos on github last year
// Things to check: ensuring all strings/options etc are fresh

// I think it has to be ensured in the crossover operation - ensuring creating a fresh one
public class FuzzingSelectionsMutation implements MutationOperator<FuzzingSelectionsSolution> {

	public enum ChangeOp {
		CHANGE_TIME_SPEC,
		CHANGE_PARAM, 
		CHANGE_PARTICIPANTS
	}
	
	public enum TimeSpecChangeOp {
		CHANGE_LENGTH,
		CHANGE_CONDITION
	}

	private final double PROB_OF_TEMPORAL_MUTATION = 1.0 / 3.0;
	private final double PROB_OF_PARAM_CHANGE = 1.0 / 3.0;
	private final double PROB_OF_TIMESPEC_CHANGE_LENGTH = 1.0 / 2.0;
	private final double DEFAULT_PROB_OF_INCLUDING_ROBOT = 0.5;
	
	private final int MAX_INDIVIDUAL_MUTATIONS = 2;
	private final double TIME_SHIFT = 500.0;

	private static final long serialVersionUID = 1L;
	private Random rng;
	private FileWriter mutationLog;
	private double mutationProb;
	protected FuzzingEngine fuzzEngine;
	protected Mission mission;
	
	FuzzingSelectionsMutation(Random rng, Mission mission, FuzzingEngine fuzzEngine, String mutationLogFileName, double mutationProb) throws IOException {
		this.rng = rng;
		this.mutationProb = mutationProb;
		this.mutationLog = new FileWriter(mutationLogFileName);
		this.fuzzEngine = fuzzEngine;
		this.mission = mission;
	}
	
	public ChangeOp selectRandomOperation() {
		double v = rng.nextDouble();
		if (v < PROB_OF_TEMPORAL_MUTATION) {
			return ChangeOp.CHANGE_TIME_SPEC;
		} else { 
			if (v < PROB_OF_TEMPORAL_MUTATION + PROB_OF_PARAM_CHANGE) {
				return ChangeOp.CHANGE_PARAM;
			} else {
				return ChangeOp.CHANGE_PARTICIPANTS;
			}
		}
	}
	
	public TimeSpecChangeOp setTimeRecord() {
		double v = rng.nextDouble();
		if (v < PROB_OF_TIMESPEC_CHANGE_LENGTH) {
			return TimeSpecChangeOp.CHANGE_LENGTH;
		} else {
			return TimeSpecChangeOp.CHANGE_CONDITION;
		}
	}

//	HashMap<MutationOperation, Double> mutationOps = new HashMap<MutationOperation, Double>();
//
//	void setupMutationOperations() {
//		mutationOps.put(new MoveTimeStart(mutationLog, TIME_SHIFT), 0.25);
//	}

	
	private void changeTimeSpec(FuzzingSelectionRecord krec) {
		
	}
	
	protected <E> E selectRandomElementFrom(List<E> l, String what) throws ListHasNoElement {
		int length = l.size();
		if (length == 0) {
			throw new ListHasNoElement(what); 
		} else {
			int i = rng.nextInt(length);
			return l.get(i);
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
	
	protected List<String> getRandomParticipantsFromMission() {
		List<String> participants = new ArrayList<String>();
		for (Robot r : mission.getAllRobots()) {
			if (rng.nextDouble() < DEFAULT_PROB_OF_INCLUDING_ROBOT) {
				participants.add(r.getName());
			}
		}
		return participants;
	}
	
	private void newParticipants(FuzzingSelectionRecord m) throws ListHasNoElement, OperationLoadFailed {
		List<String> newParticipants = getRandomParticipantsFromMission();
		((FuzzingKeySelectionRecord)m).setParticipants(newParticipants);
	}
	
	public void modifyGivenRecord(FuzzingSelectionRecord krec) {
		try {
			ChangeOp operation = selectRandomOperation();
			System.out.println("Mutation operation selected = " + operation);
			
			if (operation == ChangeOp.CHANGE_TIME_SPEC) {
				changeTimeSpec(krec);
			}

			if (operation == ChangeOp.CHANGE_PARAM) {
				newParameters(krec);
			}
			
			if (operation == ChangeOp.CHANGE_PARTICIPANTS) {
				newParticipants(krec);
			}
		} catch (ListHasNoElement e) {
			System.out.println("generateExperimentBasedUpon - empty original experiment");
		} catch (OperationLoadFailed e) {
			e.printStackTrace();
		}
	}

	private void mutatePossiblyMultipleTimes(FuzzingSelectionRecord input, int maxTimes) {
		modifyGivenRecord(input);
		int extraMutations = rng.nextInt(maxTimes);
		for (int i = 0; i < extraMutations; i++) {
			modifyGivenRecord(input);
		}
	}

	public double getMutationProbability() {
		return mutationProb;
	}

	public FuzzingSelectionsSolution execute(FuzzingSelectionsSolution source) {
		System.out.println("source=" + source);
		for (int i = 0; i < source.getNumberOfVariables(); i++) {
			FuzzingSelectionRecord fuzzingSelection = source.getVariable(i);
			mutatePossiblyMultipleTimes(fuzzingSelection, MAX_INDIVIDUAL_MUTATIONS);
			System.out.println("contents length = " + source.getNumberOfVariables());
		}
		return source;
	}
}
