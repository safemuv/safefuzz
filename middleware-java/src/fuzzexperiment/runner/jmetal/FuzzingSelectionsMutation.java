package fuzzexperiment.runner.jmetal;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.uma.jmetal.operator.mutation.MutationOperator;

import atlasdsl.Mission;
import atlasdsl.Robot;
import fuzzexperiment.runner.jmetal.grammar.Grammar;
import fuzzexperiment.runner.jmetal.grammar.GrammarBasedSubtreeMutation;
import fuzzingengine.FuzzingCondition;
import fuzzingengine.FuzzingConditionStartSpec;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingFixedTimeSpecification;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.conditionelements.FuzzingConditionElement;
import fuzzingengine.exptgenerator.ListHasNoElement;
import fuzzingengine.exptgenerator.OperationLoadFailed;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import it.units.malelab.jgea.representation.tree.Tree;

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
	//private final double PROB_OF_TIMESPEC_CHANGE_LENGTH = 1.0 / 2.0;
	private final double PROB_OF_TIMESPEC_CHANGE_LENGTH = 1.0;
	
	private final double DEFAULT_PROB_OF_INCLUDING_ROBOT = 0.5;
	
	private final int MAX_INDIVIDUAL_MUTATIONS = 2;
	private final double TIME_SHIFT = 500.0;
	
	protected final int MUTATION_DEPTH = 1;

	private static final long serialVersionUID = 1L;
	private Random rng;
	private FileWriter mutationLog;
	private double mutationProb;
	protected FuzzingEngine fuzzEngine;
	protected Mission mission;
	protected GrammarBasedSubtreeMutation mutator;
	
	
	FuzzingSelectionsMutation(Grammar g, Random rng, Mission mission, FuzzingEngine fuzzEngine, String mutationLogFileName, double mutationProb) throws IOException {
		this.rng = rng;
		this.mutationProb = mutationProb;
		this.mutationLog = new FileWriter(mutationLogFileName);
		this.fuzzEngine = fuzzEngine;
		this.mission = mission;
		this.mutator = new GrammarBasedSubtreeMutation(MUTATION_DEPTH, g);
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
	
	public TimeSpecChangeOp getTimeSpecChangeOperation() {
		double v = rng.nextDouble();
		if (v < PROB_OF_TIMESPEC_CHANGE_LENGTH) {
			return TimeSpecChangeOp.CHANGE_LENGTH;
		} else {
			return TimeSpecChangeOp.CHANGE_CONDITION;
		}
	}

	private void changeTimeSpec(FuzzingSelectionRecord krec) {
		FuzzingTimeSpecification ts = krec.getTimeSpec();
		if (ts instanceof FuzzingFixedTimeSpecification) {
			
		}
		
		if (ts instanceof FuzzingConditionStartSpec) {
			TimeSpecChangeOp changeOp = getTimeSpecChangeOperation();
			if (changeOp == TimeSpecChangeOp.CHANGE_CONDITION) {
				FuzzingConditionStartSpec tsc = (FuzzingConditionStartSpec)ts;
				FuzzingCondition c = tsc.getCondition();
				Tree<FuzzingConditionElement> t = c.getTree();
				
				// Mutate the condition tree here
				FuzzingConditionStartSpec newC = new FuzzingConditionStartSpec(c, tsc.getEndTime());
				krec.setTimeSpec(newC);
			}
		}
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
			System.out.println("fuzzingSelection=" + fuzzingSelection);
			mutatePossiblyMultipleTimes(fuzzingSelection, MAX_INDIVIDUAL_MUTATIONS);
			System.out.println("contents length = " + source.getNumberOfVariables());
		}
		return source;
	}
}
