package exptrunner.jmetal;

import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Random;

import org.uma.jmetal.operator.mutation.MutationOperator;

import fuzzingengine.FuzzingSelectionRecord;
import exptrunner.operations.*;

// TODO: check mutation logic against the changes that I made with Simos on github last year
// Things to check: ensuring all strings/options etc are fresh
public class FuzzingSelectionsMutation implements MutationOperator<FuzzingSelectionsSolution> {
		
		private final int MAX_INDIVIDUAL_MUTATIONS = 2;
		private final double TIME_SHIFT = 500.0;

		private static final long serialVersionUID = 1L;

		private Random rng;
		private FileWriter mutationLog;
		private double mutationProb;
		
		HashMap<MutationOperation,Double> mutationOps = new HashMap<MutationOperation,Double>();

		void setupMutationOperations() {
			mutationOps.put(new MoveTimeStart(mutationLog, TIME_SHIFT), 0.25);
		}
		
		FuzzingSelectionsMutation(Random rng, String mutationLogFileName, double mutationProb) throws IOException {
			this.rng = rng;
			this.mutationProb = mutationProb;
			this.mutationLog = new FileWriter(mutationLogFileName);
		}

		private MutationOperation chooseMutationOption() {
			// TODO: return one mutation accumulation
			return (MutationOperation) mutationOps.keySet().toArray()[0];
		}
		
		// TODO: split up this logic into files in exptrunner.operations
		private void mutateFuzzingSelectionRandomly(FuzzingSelectionRecord fi) {
			MutationOperation mutationOp = chooseMutationOption();
			System.out.println("Performing mutation " + mutationOp.name() + " on fault instance " + fi.toString());
			try {
				mutationLog.write("Performing mutation on fault instance " + fi.toString() + "\n");
				mutationOp.perform(fi);
				fi.checkConstraints();
				System.out.println("Mutated fault = " + fi.toString());
				mutationLog.write("Mutated fault = " + fi.toString() + "\n");
				mutationLog.flush();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		private void mutatePossiblyMultipleTimes(FuzzingSelectionRecord input, int maxTimes) {
			mutateFuzzingSelectionRandomly(input);
			int extraMutations = rng.nextInt(maxTimes);
			for (int i = 0; i < extraMutations; i++) {
				mutateFuzzingSelectionRandomly(input);
			}
		}

		public double getMutationProbability() {
			return mutationProb;
		}

		public FuzzingSelectionsSolution execute(FuzzingSelectionsSolution source) {
			for (int i = 0; i < source.getNumberOfVariables(); i++) {
				FuzzingSelectionRecord fuzzingSelection = source.getVariable(i);
				mutatePossiblyMultipleTimes(fuzzingSelection, MAX_INDIVIDUAL_MUTATIONS);
				System.out.println("contents length = " + source.getNumberOfVariables());
			}
			return source;
		}
	}
