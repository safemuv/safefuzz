package exptrunner;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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
	
	private void setupInitialPopulation() {
		pop.clear();
		popResults.clear();
		runNoInPopulation = 0;
		List<Fault> allFaults = mission.getFaultsAsList();
		Fault f = randomElementInList(allFaults);
		for (int i = 0; i < INITIAL_POPULATION_SIZE; i++) {
			pop.add(i, new FaultInstanceSet(index -> newFaultInstance(f), NUMBER_OF_INITIAL_FAULTS));
		}
	}
	
	private int scoreForResult(FaultInstanceSet fs, ResultInfo r) {
		return r.getTotalFaults();
	}

	private Stream<Entry<FaultInstanceSet, Integer>> rankPopulation() {
		return popResults
				.entrySet().stream()
				.collect(Collectors.toMap(Map.Entry::getKey, e -> scoreForResult(e.getKey(), e.getValue())))
				.entrySet().stream()
				.sorted(Map.Entry.comparingByValue());
	}
		
	// Mutates the population
	private void mutatePopulation(List<FaultInstanceSet> population, FaultInstanceSet best) {
		// TODO: select the best few as a basis for mutation
		// TODO: specify a mutation function 
		population.stream().map(fis -> fis.mutateWithProb(MUTATION_PROB, mut -> mut));
	}
	
	private void iteratePopulation() {
		iteration++;
		// TODO: log the population to a file
		Stream<Entry<FaultInstanceSet, Integer>> ranks = rankPopulation();
		Optional<Entry<FaultInstanceSet, Integer>> bestElement = ranks.findFirst();
		
		if (bestElement.isPresent()) {
			FaultInstanceSet best = bestElement.get().getKey();
			mutatePopulation(pop, best);
		}
	}
	
	public FaultMutation(String resFileName, int seed, int maxIterationCount, Mission mission) throws IOException {
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
		// create a new ResultInfo and store it
		FaultInstanceSet currentFS = pop.get(runNoInPopulation);
		ResultInfo ri = new ResultInfo();
		popResults.put(currentFS, ri);
		// TODO: logging to files
	}

	public void printState() {
		System.out.println("iterations = " + iteration);
	}
}
