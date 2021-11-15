package fuzzexperiment.runner.jmetal;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;

import org.uma.jmetal.operator.mutation.MutationOperator;

import atlasdsl.Mission;
import atlasdsl.Robot;
import fuzzexperiment.runner.jmetal.grammar.Grammar;
import fuzzexperiment.runner.jmetal.grammar.GrammarBasedSubtreeMutation;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingFixedTimeSpecification;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.TimeSpec;
import fuzzingengine.exptgenerator.ListHasNoElement;
import fuzzingengine.exptgenerator.OperationLoadFailed;
import fuzzingengine.grammar.FuzzingCondition;
import fuzzingengine.grammar.FuzzingConditionStartEnd;
import fuzzingengine.grammar.FuzzingConditionStartSpec;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import it.units.malelab.jgea.representation.tree.Tree;

// TODO: check mutation logic against the changes that I made with Simos on github last year
// Things to check: ensuring all strings/options etc are fresh

// I think it has to be ensured in the crossover operation - ensuring creating a fresh one
public class FuzzingSelectionsMutation implements MutationOperator<FuzzingSelectionsSolution> {

	public enum TimeSpecChangeOp {
		CHANGE_LENGTH, CHANGE_START_CONDITION, CHANGE_END_CONDITION
	}

	protected final double DEFAULT_PROB_OF_TEMPORAL_MUTATION = 1.0 / 3.0;
	protected final double DEFAULT_PROB_OF_PARAM_CHANGE = 1.0 / 3.0;
	protected final double DEFAULT_PROB_OF_PARTICIPANT_CHANGE = 1.0 / 3.0;
	
	private static final boolean DELETE_OVERLAPPING_RECORDS = false;

	protected double probTemporalMutation = DEFAULT_PROB_OF_TEMPORAL_MUTATION;
	protected double probParamMutation = DEFAULT_PROB_OF_PARAM_CHANGE;
	protected double probParticipantMutation = DEFAULT_PROB_OF_PARTICIPANT_CHANGE;

	private final double PROB_OF_TIMESPEC_CHANGE_END = 0.5;

	private final double DEFAULT_PROB_OF_INCLUDING_ROBOT = 0.5;

	protected final int MUTATION_DEPTH = 1;

	private static final long serialVersionUID = 1L;
	
	
	
	
	protected Random rng;
	private FileWriter mutationLog;
	protected FuzzingEngine fuzzEngine;
	protected Mission mission;
	protected GrammarBasedSubtreeMutation<String> mutator;

	FuzzingSelectionsMutation(Grammar g, Random rng, Mission mission, FuzzingEngine fuzzEngine,
			String mutationLogFileName) throws IOException {
		this.rng = rng;
		this.mutationLog = new FileWriter(mutationLogFileName);
		this.fuzzEngine = fuzzEngine;
		this.mission = mission;
		this.mutator = new GrammarBasedSubtreeMutation<String>(MUTATION_DEPTH, g);
	}

	FuzzingSelectionsMutation(Grammar g, Random rng, Mission mission, FuzzingEngine fuzzEngine,
			String mutationLogFileName, double probTemporalMut, double probParamMut, double probParticipantMut)
			throws IOException {
		this.rng = rng;
		this.probTemporalMutation = probTemporalMut;
		this.probParamMutation = probParamMut;
		this.probParticipantMutation = probParticipantMut;

		this.mutationLog = new FileWriter(mutationLogFileName);
		mutationLog.write("probTemporalMutation=" + probTemporalMut + "\n");
		mutationLog.write("probParamMutation=" + probParamMut + "\n");
		mutationLog.write("probParticipantMutation=" + probParticipantMut + "\n\n");

		this.fuzzEngine = fuzzEngine;
		this.mission = mission;
		this.mutator = new GrammarBasedSubtreeMutation<String>(MUTATION_DEPTH, g);
	}

	// This only applies to the start condition + time length
	public TimeSpecChangeOp getTimeSpecChangeOperationStartLength() {
		double v = rng.nextDouble();
		if (v < PROB_OF_TIMESPEC_CHANGE_END) {
			return TimeSpecChangeOp.CHANGE_LENGTH;
		} else {
			return TimeSpecChangeOp.CHANGE_START_CONDITION;
		}
	}

	// This only applies to the start condition + end condition
	public TimeSpecChangeOp getTimeSpecChangeOperationStartEnd() {
		double v = rng.nextDouble();
		if (v < PROB_OF_TIMESPEC_CHANGE_END) {
			return TimeSpecChangeOp.CHANGE_START_CONDITION;
		} else {
			return TimeSpecChangeOp.CHANGE_END_CONDITION;
		}
	}

	protected double getStartTime(Optional<TimeSpec> ts_o) {
		double startLimit;

		if (ts_o.isPresent()) {
			startLimit = ts_o.get().getEndTime();
		} else {
			startLimit = mission.getEndTime();
		}

		return startLimit * rng.nextDouble();
	}

	protected double getEndTime(Optional<TimeSpec> ts_o, double startTime) {
		double endLimit;

		if (ts_o.isPresent()) {
			endLimit = ts_o.get().getEndTime();
		} else {
			endLimit = mission.getEndTime();
		}

		double endTime = (endLimit - startTime) * rng.nextDouble() + startTime;
		return endTime;
	}

	private void changeFixedTimeSpec(FuzzingSelectionRecord krec) {
		double startTime;
		double endTime;
		FuzzingSimMapping fsm = fuzzEngine.getSimMapping();

		if (krec instanceof FuzzingKeySelectionRecord) {
			FuzzingKeySelectionRecord km = (FuzzingKeySelectionRecord) krec;
			VariableSpecification vs = fsm.getRecords().get(km.getKey());
			Optional<TimeSpec> ts = vs.getTimeSpec();
			startTime = getStartTime(ts);
			endTime = getEndTime(ts, startTime);
			FuzzingFixedTimeSpecification newT = new FuzzingFixedTimeSpecification(startTime, endTime);
			krec.setTimeSpec(newT);
		} else {
			logWithoutError("changeFixedTimeSpec - called on record that is not key selection record");
		}
	}

	private void changeTimeSpec(FuzzingSelectionRecord krec) {
		FuzzingTimeSpecification ts = krec.getTimeSpec();
		if (ts instanceof FuzzingFixedTimeSpecification) {
			changeFixedTimeSpec(krec);
		}

		if (ts instanceof FuzzingConditionStartSpec) {
			TimeSpecChangeOp changeOp = getTimeSpecChangeOperationStartLength();
			if (changeOp == TimeSpecChangeOp.CHANGE_START_CONDITION) {
				FuzzingConditionStartSpec tsc = (FuzzingConditionStartSpec) ts;
				FuzzingCondition c = tsc.getCondition();
				Tree<String> t = c.getTree();
				Tree<String> tNew = mutator.mutate(t, rng);

				System.out.print("MUTATION: Original tree = ");
				t.prettyPrintLine(System.out);
				System.out.print(": Mutated tree = ");
				tNew.prettyPrintLine(System.out);
				System.out.print("\n");

				FuzzingCondition cNew = new FuzzingCondition(tNew);
				FuzzingConditionStartSpec newC = new FuzzingConditionStartSpec(cNew, tsc.getEndTime());
				krec.setTimeSpec(newC);
			}
		}

		if (ts instanceof FuzzingConditionStartEnd) {
			TimeSpecChangeOp changeOp = getTimeSpecChangeOperationStartEnd();
			FuzzingConditionStartEnd tsc = (FuzzingConditionStartEnd) ts;

			if (changeOp == TimeSpecChangeOp.CHANGE_START_CONDITION) {
				// CHANGE START CONDITION
				FuzzingCondition c = tsc.getStartCondition();
				Tree<String> t = c.getTree();
				Tree<String> tNew = mutator.mutate(t, rng);

				System.out.print("MUTATION: Original tree = ");
				t.prettyPrintLine(System.out);
				System.out.print(": Mutated tree = ");
				tNew.prettyPrintLine(System.out);
				System.out.print("\n");

				FuzzingCondition cNew = new FuzzingCondition(tNew);
				FuzzingConditionStartEnd newC = new FuzzingConditionStartEnd(cNew, tsc.getEndCondition());
				krec.setTimeSpec(newC);
			}

			if (changeOp == TimeSpecChangeOp.CHANGE_END_CONDITION) {
				// CHANGE END CONDITION
				FuzzingCondition c = tsc.getEndCondition();
				Tree<String> t = c.getTree();
				Tree<String> tNew = mutator.mutate(t, rng);

				System.out.print("MUTATION: Original tree = ");
				t.prettyPrintLine(System.out);
				System.out.print(": Mutated tree = ");
				tNew.prettyPrintLine(System.out);
				System.out.print("\n");

				FuzzingCondition cNew = new FuzzingCondition(tNew);
				FuzzingConditionStartEnd newC = new FuzzingConditionStartEnd(tsc.getStartCondition(), cNew);
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
				FuzzingKeySelectionRecord kr = (FuzzingKeySelectionRecord) m;
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
		((FuzzingKeySelectionRecord) m).setParticipants(newParticipants);
	}

	public void logWithoutError(String s) {
		try {
			mutationLog.write(s + "\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void modifyGivenRecord(FuzzingSelectionRecord krec) {
		try {
			if (rng.nextDouble() < probTemporalMutation) {
				logWithoutError("Performing temporal mutation on " + krec.getSimpleName());
				changeTimeSpec(krec);
			}

			if (rng.nextDouble() < probParamMutation) {
				logWithoutError("Performing parameter mutation on " + krec.getSimpleName());
				newParameters(krec);
			}

			if (rng.nextDouble() < probParticipantMutation) {
				logWithoutError("Performing participant mutation on " + krec.getSimpleName());
				newParticipants(krec);
			}

		} catch (ListHasNoElement e) {
			System.out.println("generateExperimentBasedUpon - empty original experiment");
		} catch (OperationLoadFailed e) {
			e.printStackTrace();
		}
	}

	public double getMutationProbability() {
		return 1.0;
	}

	public FuzzingSelectionsSolution execute(FuzzingSelectionsSolution sol) {
		// PRE-MUTATION DEBUGGING
		try {
			mutationLog.write(
					"---------------------------------------------------------------------------------------------------\n");
			System.out.println("Performing mutation: source=" + sol.getCSVFileName());
			mutationLog.write("Performing mutation: source=" + sol.getCSVFileName() + "\n");
			sol.printCSVContentsToFile(mutationLog);
			mutationLog.write("\n");
		} catch (IOException e) {
			e.printStackTrace();
		}

		for (int i = 0; i < sol.getNumberOfVariables(); i++) {
			FuzzingSelectionRecord fuzzingSelection = sol.getVariable(i);
			System.out.println("fuzzingSelection=" + fuzzingSelection);
			modifyGivenRecord(fuzzingSelection);
			try {
				sol.regenerateCSVFile();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		// POST-MUTATION DEBUGGING

		if (DELETE_OVERLAPPING_RECORDS) {
			try {
				System.out.println("After mutation: source=" + sol.getCSVFileName());
				mutationLog.write("After mutation: source=" + sol.getCSVFileName() + "\n");
				sol.printCSVContentsToFile(mutationLog);
				mutationLog.write("\n");

				mutationLog.write(
						"---------------------------------------------------------------------------------------------------\n");
				mutationLog.write("Deleting overlapping records produced: \n");
				// Now, check the solution for overlapping elements
				sol.deleteOverlapping();
				sol.printCSVContentsToFile(mutationLog);
				mutationLog.write("Deleting overlapping records done\n");
				mutationLog.write(
						"---------------------------------------------------------------------------------------------------\n");
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		return sol;
	}

	public void closeLog() throws IOException {
		mutationLog.close();
	}
}
