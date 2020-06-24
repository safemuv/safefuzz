package exptrunner;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.Random;
import java.util.Scanner;
import java.util.stream.Collectors;
import static java.util.Collections.reverseOrder;

import atlasdsl.Mission;
import atlasdsl.faults.Fault;
import atlassharedclasses.FaultInstance;

public class FaultMutation extends ExptParams {

	// The number of fault instances produced in the initial sets of the population
	private static final int NUMBER_OF_INITIAL_FAULTS = 3;

	// The number of randomly generated test fault instance sets in the population
	private static final int INITIAL_POPULATION_SIZE = 3;

	// Probability of mutation of existing faults
	private static final double MUTATION_PROB = 0.3;

	private static final int MUTATIONS_OF_SELECTED_BEST = 2;
	private static final int EXISTING_POP_TO_RETAIN = 1;

	private static final int DETECTIONS_PER_OBJECT = 2;

	private static final int MAX_INDIVIDUAL_MUTATIONS = 2;

	private List<FaultInstanceSet> pop = new ArrayList<FaultInstanceSet>();
	private Map<FaultInstanceSet, ResultInfo> popResults = new LinkedHashMap<FaultInstanceSet, ResultInfo>();
	private Map<FaultInstanceSet, ResultInfo> storedResults = new LinkedHashMap<FaultInstanceSet, ResultInfo>();

	private FileWriter evolutionLog;
	private int maxIterationCount;

	private int runNoInPopulation = 0;
	private int iteration = 0;

	private Mission mission;
	private Random r;

	private int envObjectCount;

	private double totalFaultTimeInModel = 0.0;

	private enum MutationType {
		EXPAND_LENGTH, CONTRACT_LENGTH, CHANGE_ADDITIONAL_INFO, MOVE_START;
	}

	// Overall algorithm...

	// Run all the simulations for the current population - store the results for
	// each one
	// When the end point is reached... rank them all
	// Ranking criteria: what is the analysis for them all?

	public FaultMutation(String resFileName, double runTime, long seed, int maxIterationCount, Mission mission)
			throws IOException {
		super(runTime);
		this.evolutionLog = new FileWriter(resFileName);
		this.r = new Random(seed);
		this.runNoInPopulation = 0;
		this.mission = mission;
		this.maxIterationCount = maxIterationCount;
		setupInitialPopulation();
		this.envObjectCount = mission.getEnvironmentalObjects().size();

		// TODO: this needs to be set
		this.totalFaultTimeInModel = 0.0;
	}

	private FaultInstance newFaultInstance(Fault f) {
		double maxRange = f.getLatestEndTime() - f.getEarliestStartTime();
		double timeStart = f.getEarliestStartTime() + r.nextDouble() * maxRange;

		double rangeOfEnd = f.getLatestEndTime() - timeStart;
		double timeEnd = timeStart + r.nextDouble() * rangeOfEnd;

		// TODO: optional intensity data
		return new FaultInstance(timeStart, timeEnd, f, Optional.empty());
	}

	private <T> T randomElementInList(List<T> l) {
		int i = r.nextInt(l.size());
		return l.get(i);
	}

	private void debugPrintPopulation(List<FaultInstanceSet> pop) {
		int count = 0;
		for (FaultInstanceSet fs : pop) {
			System.out.println(count + "-" + fs.toString() + "\n");
			System.out.println("Fault instance set contains: " + fs.getCount() + " faults\n");
			count++;
		}
	}

	private void debugPrintPopulationWithRanks(List<Entry<FaultInstanceSet, Double>> ranks) {
		for (Map.Entry<FaultInstanceSet, Double> e : ranks) {
			System.out.println(e.getKey().toString() + " - rank " + e.getValue());
		}
	}

	private void setupInitialPopulation() {
		System.out.println("Setting up initial population...");
		pop.clear();
		popResults.clear();
		runNoInPopulation = 0;
		List<Fault> allFaults = mission.getFaultsAsList();
		for (int i = 0; i < INITIAL_POPULATION_SIZE; i++) {
			pop.add(i, new FaultInstanceSet(index -> {
				Fault f = randomElementInList(allFaults);
				return newFaultInstance(f);
			}, NUMBER_OF_INITIAL_FAULTS));
		}
		// print the initial population
		debugPrintPopulation(pop);
	}

	private double scoreForResult(FaultInstanceSet fs, ResultInfo ri) {
		// Score based on the size of the total fault length too!
		// TODO: compute total fault length possible in model! This will give us a
		// metric that is 0 -> 1,
		// so therefore will be used as a secondary factor
		double score = ri.getTotalFaults() + (1 - ((fs.totalFaultTimeLength() / (fs.getCount() * runtime))));
		System.out.println("scoreForResult: total faults=" + ri.getTotalFaults());
		System.out.println("scoreForResult: time length =" + fs.totalFaultTimeLength());
		return score;
	}

	private List<Entry<FaultInstanceSet, Double>> rankPopulation() {
		return popResults.entrySet().stream()
				.collect(Collectors.toMap(Map.Entry::getKey, e -> scoreForResult(e.getKey(), e.getValue()))).entrySet()
				.stream().sorted(reverseOrder(Map.Entry.comparingByValue())).collect(Collectors.toList());
	}

	// Choose the mutation options with equal probability
	private MutationType chooseMutationOption() {
		double v = r.nextDouble();
		if (v < 0.333) {
			return MutationType.CONTRACT_LENGTH;
		} else if (v < 0.666) {
			return MutationType.EXPAND_LENGTH;
		} else
			return MutationType.MOVE_START;
	}

	private FaultInstance mutateFaultInstanceRandomly(FaultInstance input) {
		FaultInstance output = new FaultInstance(input);
		double maxTimeShift = input.getFault().getMaxTimeRange();
		MutationType mutationType = chooseMutationOption();
		System.out.println("Performing mutation on fault instance " + input.toString());

		switch (mutationType) {
		case CONTRACT_LENGTH:
			// Type of mutation: contracting an FI - reducing its length
			double expandFactor = r.nextDouble();
			System.out.println("Contracting length: factor = " + expandFactor);
			output.multLengthFactor(expandFactor);
		case EXPAND_LENGTH:
			// Type of mutation: expanding an FI temporally - extending its length
			double contractFactor = r.nextDouble();
			expandFactor = 1.0 / contractFactor;
			System.out.println("Expanding length: factor = " + expandFactor);
			output.multLengthFactor(expandFactor);

		case MOVE_START:
			double absTimeShift = (r.nextDouble() - 0.5) * maxTimeShift * 2;
			System.out.println("Moving fault: absTimeShift = " + absTimeShift);
			output.absShiftTimes(absTimeShift);
		}
		System.out.println("mutated fault = " + output.toString());
		return output;
	}

	private FaultInstance mutatePossiblyMultipleTimes(FaultInstance input, int maxTimes) {
		FaultInstance mutated = mutateFaultInstanceRandomly(input);
		int extraMutations = r.nextInt(maxTimes);
		for (int i = 0; i < extraMutations; i++) {
			mutated = mutateFaultInstanceRandomly(mutated);
		}
		return mutated;
	}

	private List<FaultInstanceSet> mutatePopulation(List<FaultInstanceSet> population,
			List<Entry<FaultInstanceSet, Double>> ranks) {

		List<FaultInstanceSet> newPop = new ArrayList<FaultInstanceSet>();

		// create new populations by mutating best one
		for (int i = 0; i < MUTATIONS_OF_SELECTED_BEST; i++) {
			FaultInstanceSet best = ranks.get(0).getKey();
			newPop.add(best);
			System.out.println("Best chosen:=" + best.hashCode() + ", score = " + ranks.get(0).getValue());
			// TODO: specify a mutation function for the individual fault instances
			FaultInstanceSet mutatedBest = best.mutateWithProb(r, MUTATION_PROB,
					fi -> mutatePossiblyMultipleTimes(fi, MAX_INDIVIDUAL_MUTATIONS));
			newPop.add(mutatedBest);
		}

		for (int i = 0; i <= EXISTING_POP_TO_RETAIN; i++) {
			FaultInstanceSet existing = ranks.get(i).getKey();
			newPop.add(existing);
		}

		return newPop;
	}

	private void iteratePopulation() {
		iteration++;
		List<Entry<FaultInstanceSet, Double>> ranks = rankPopulation();
		pop = mutatePopulation(pop, ranks);
		debugPrintPopulationWithRanks(ranks);
	}

	public void advance() {
		runNoInPopulation++;
		if (runNoInPopulation >= pop.size()) {
			runNoInPopulation = 0;
			iteratePopulation();
		}
	}

	public List<FaultInstance> specificFaults() {
		return pop.get(runNoInPopulation).asList();
	}

	private String specificFaultsAsString() {
		List<FaultInstance> fis = specificFaults();
		String str = fis.stream().map(f -> f.toString()).collect(Collectors.joining());
		return str;
	}

	public boolean completed() {
		return (iteration > maxIterationCount);
	}

	private void countDetections(String logFileDir, ResultInfo ri) {
		File f = new File(logFileDir + "/goalLog.log");
		int detections = 0;
		Scanner reader;
		try {
			reader = new Scanner(f);
			while (reader.hasNextLine()) {
				String line = reader.nextLine();
				String[] fields = line.split(",");
				String goalClass = fields[0];
				String time = fields[1];
				String robot = fields[2];
				String num = fields[3];
				if (goalClass.equals("atlasdsl.DiscoverObjects")) {
					detections++;
				}
			}
			int totalExpected = envObjectCount * DETECTIONS_PER_OBJECT;
			int missedDetections = totalExpected - detections;
			ri.setField("missedDetections", missedDetections);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}

	public void logResults(String logFileDir) {
		// process the result file and obtain the results
		FaultInstanceSet currentFS = pop.get(runNoInPopulation);
		ResultInfo ri = new ResultInfo();
		countDetections(logFileDir, ri);
		popResults.put(currentFS, ri);
		// storedResults maintains a previous results cache
		// ensure it is used rather than re-evaluating!
		storedResults.put(currentFS, ri);
		// TODO: logging to additional files?
	}

	public void printState() {
		System.out.println("iterations = " + iteration + ",numInPop = " + runNoInPopulation);
	}
}
