package exptrunner;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.Random;
import java.util.stream.Collectors;

import atlasdsl.Mission;
import atlasdsl.faults.Fault;
import atlassharedclasses.FaultInstance;

public class FaultMutation extends ExptParams {
	
	// The number of fault instances produced in the initial sets of the population
	private static final int NUMBER_OF_INITIAL_FAULTS = 3;
	
	// The number of randomly generated test fault instance sets in the population
	private static final int INITIAL_POPULATION_SIZE = 10;
	
	// Probability of mutation of existing faults
	private static final double MUTATION_PROB = 0.3;

	private static final int MUTATIONS_OF_SELECTED_BEST = 5;
	private static final int EXISTING_POP_TO_RETAIN = 5;
	
	private List<FaultInstanceSet> pop = new ArrayList<FaultInstanceSet>();
	private Map<FaultInstanceSet,ResultInfo> popResults = new LinkedHashMap<FaultInstanceSet,ResultInfo>();

	private FileWriter evolutionLog;
	private int maxIterationCount;
	
	private int runNoInPopulation = 0;
	private int iteration = 0;
		
	private Mission mission;
	private Random r;
		
	// Overall algorithm...
	
	// Run all the simulations for the current population - store the results for each one
	// When the end point is reached... rank them all 
	// Ranking criteria: what is the analysis for them all?
	
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
			System.out.println(count + "-" + fs.toString());
		}
	}
	
	private void debugPrintPopulationWithRanks(List<Entry<FaultInstanceSet, Integer>> ranks) {
		for (Map.Entry<FaultInstanceSet, Integer> e : ranks) {
			System.out.println(e.getKey().toString() + " - rank " + e.getValue());
		}
	}
	
	private void setupInitialPopulation() {
		pop.clear();
		popResults.clear();
		runNoInPopulation = 0;
		List<Fault> allFaults = mission.getFaultsAsList();
		Fault f = randomElementInList(allFaults);
		for (int i = 0; i < INITIAL_POPULATION_SIZE; i++) {
			pop.add(i, new FaultInstanceSet(index -> newFaultInstance(f), NUMBER_OF_INITIAL_FAULTS));
		}
		// print the initial population
		debugPrintPopulation(pop);
	}
	
	private int scoreForResult(FaultInstanceSet fs, ResultInfo r) {
		return r.getTotalFaults();
	}

	private List<Entry<FaultInstanceSet, Integer>> rankPopulation() {
		return popResults
				.entrySet().stream()
				.collect(Collectors.toMap(Map.Entry::getKey, e -> scoreForResult(e.getKey(), e.getValue())))
				.entrySet().stream()
				.sorted(Map.Entry.comparingByValue())
				.collect(Collectors.toList());
	}
	
	private FaultInstance mutateFaultInstance(FaultInstance fi) {
		// TODO: mutate an individual fault instance
		//FaultInstance fi = fi.clone();
		//fi.adjust
		return fi;
	}
		
	private List<FaultInstanceSet> mutatePopulation(List<FaultInstanceSet> population, List<Entry<FaultInstanceSet, Integer>> ranks) {
		
		List<FaultInstanceSet> newPop = new ArrayList<FaultInstanceSet>();
		
		// create new populations by mutating best
		for (int i = 0; i < MUTATIONS_OF_SELECTED_BEST; i++) {
			FaultInstanceSet best = ranks.get(0).getKey();
			// TODO: specify a mutation function for the individual fault instances 
			FaultInstanceSet mutatedBest = best.mutateWithProb(MUTATION_PROB, fi -> mutateFaultInstance(fi));
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
		List<Entry<FaultInstanceSet, Integer>> ranks = rankPopulation();
		pop = mutatePopulation(pop, ranks);
		debugPrintPopulationWithRanks(ranks);
	}
	
	public FaultMutation(String resFileName, long seed, int maxIterationCount, Mission mission) throws IOException {
		this.evolutionLog = new FileWriter(resFileName);
		this.r = new Random(seed);
		this.runNoInPopulation = 0;
		this.mission = mission;
		this.maxIterationCount = maxIterationCount;
		setupInitialPopulation();
	}

	public void advance() {
		runNoInPopulation++;
		if (runNoInPopulation > pop.size()) {
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

	public void logResults(String logFileDir) {
		// process the result file and obtain the results
		// TODO: create a new ResultInfo and store it
		FaultInstanceSet currentFS = pop.get(runNoInPopulation);
		ResultInfo ri = new ResultInfo();
		popResults.put(currentFS, ri);
		// TODO: logging to files
	}

	public void printState() {
		System.out.println("iterations = " + iteration + ",numInPop = " + runNoInPopulation);
	}
}
