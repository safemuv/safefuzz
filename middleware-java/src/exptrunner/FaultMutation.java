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
import java.util.Set;
import java.util.stream.Collectors;
import static java.util.Collections.reverseOrder;

import atlasdsl.Mission;
import atlasdsl.faults.Fault;
import atlassharedclasses.FaultInstance;

public class FaultMutation extends ExptParams {

	// The number of fault instances produced in the initial sets of the population
	// private static final int NUMBER_OF_INITIAL_FAULTS = 8;
	// We assume that there is one fault instance per fault in the model

	// The number of randomly generated test fault instance sets in the population
	private static final int INITIAL_POPULATION_SIZE = 8;

	// Probability of mutation of existing faults
	private static final double MUTATION_PROB = 0.3;

	private static final int MUTATIONS_OF_SELECTED_BEST = 6;
	private static final int EXISTING_POP_TO_RETAIN = 2;

	private static final int DETECTIONS_PER_OBJECT = 2;

	private static final int MAX_INDIVIDUAL_MUTATIONS = 3;

	private static final double INACTIVE_INITIAL_FAULT_PROB = 0.7;

	private static final double MIN_SPEED_VALUE = 2.0;
	private static final double MAX_SPEED_VALUE = 5.0;

	// TODO: constant probabilities for the mutation process

	private List<FaultInstanceSet> pop = new ArrayList<FaultInstanceSet>();
	private Map<FaultInstanceSet, ResultInfo> popResults = new LinkedHashMap<FaultInstanceSet, ResultInfo>();
	private Map<FaultInstanceSet, ResultInfo> storedResults = new LinkedHashMap<FaultInstanceSet, ResultInfo>();

	private LoggingOperation logOp;

	private FileWriter evolutionLog;
	private FileWriter mutationLog;
	private FileWriter populationLog;

	private int maxIterationCount;

	private int runNoInPopulation = 0;
	private int iteration = 0;

	private Mission mission;
	private Random r;

	private int envObjectCount;

	private enum MutationType {
		EXPAND_LENGTH, CONTRACT_LENGTH, CHANGE_ADDITIONAL_INFO, MOVE_START, FLIP_ACTIVE_FLAG;
	}

	// Overall algorithm...

	// Run all the simulations for the current population - store the results for
	// each one
	// When the end point is reached... rank them all
	// Ranking criteria: what is the analysis for them all?
	public FaultMutation(LoggingOperation logOp, String resFileName, String mutationFileName, String populationFileName,
			double runTime, long seed, int maxIterationCount, Mission mission) throws IOException {
		super(runTime);
		this.logOp = logOp;
		this.evolutionLog = new FileWriter(resFileName);
		this.mutationLog = new FileWriter(mutationFileName);
		this.populationLog = new FileWriter(populationFileName);
		this.r = new Random(seed);
		this.runNoInPopulation = 0;
		this.mission = mission;
		this.maxIterationCount = maxIterationCount;
		setupInitialPopulation();
		this.envObjectCount = mission.getEnvironmentalObjects().size();
	}

	private FaultInstance newFaultInstance(Fault f) {
		double maxRange = f.getLatestEndTime() - f.getEarliestStartTime();
		double timeStart = f.getEarliestStartTime() + r.nextDouble() * maxRange;

		double rangeOfEnd = f.getLatestEndTime() - timeStart;
		double timeEnd = timeStart + r.nextDouble() * rangeOfEnd;
		

		FaultInstance fi = new FaultInstance(timeStart, timeEnd, f, Optional.empty());
		
		if (r.nextDouble() < INACTIVE_INITIAL_FAULT_PROB) {
			fi.setActiveFlag(false);
		}
		
		return changeAdditionalInfo(fi);
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
		getPop().clear();
		getPopResults().clear();
		runNoInPopulation = 0;
		List<Fault> allFaults = mission.getFaultsAsList();
		
		for (int i = 0; i < INITIAL_POPULATION_SIZE; i++) {
			getPop().add(i, new FaultInstanceSet(index -> {
				Fault f = allFaults.get(index);
				FaultInstance fi = newFaultInstance(f);
				return fi;
			}, allFaults.size()));
		}
		// print the initial population
		debugPrintPopulation(getPop());
	}
	
	private double totalActiveFaultTimeLengthScaledByIntensity(FaultInstanceSet fs) {
		Set<FaultInstance> faultInstances = fs.getFaultInstances();
		double total = 0.0;
		for (FaultInstance fi : faultInstances) {
			if (fi.isActive()) {
				total += faultInstanceIntensity(fi) * (fi.getEndTime() - fi.getStartTime());
			}
		}
		return total;
	}

	private double faultCostProportion(FaultInstanceSet fs, ResultInfo ri) {
		
		return 1 - ((totalActiveFaultTimeLengthScaledByIntensity(fs) / (fs.getCount() * runtime)));
	}

	private double scoreForResult(FaultInstanceSet fs, ResultInfo ri) {
		// Score based on the size of the total fault length too!
		// Compute total fault length possible in model! This will give us a
		// metric that is 0 -> 1,
		// so therefore will be used as a secondary factor
		double fc = faultCostProportion(fs, ri);
		double totalFaults = ri.getTotalGoalViolations();
		double score = totalFaults + fc;
		System.out.println("scoreForResult:goal violations=" + ri.getTotalGoalViolations());
		System.out.println("scoreForResult: time length   =" + fs.totalFaultTimeLength());
		System.out.println("scoreForResult: final score   =" + score);
		return score;
	}

	private List<Entry<FaultInstanceSet, Double>> rankPopulation() {
		return getPopResults().entrySet().stream()
				.collect(Collectors.toMap(Map.Entry::getKey, e -> scoreForResult(e.getKey(), e.getValue()))).entrySet()
				.stream().sorted(reverseOrder(Map.Entry.comparingByValue())).collect(Collectors.toList());
	}

	// Active flag flipping - most probable!
	private MutationType chooseMutationOption() {
		double v = r.nextDouble();
		if (v < 0.15) {
			return MutationType.CONTRACT_LENGTH;
		} else if (v < 0.3) {
			return MutationType.EXPAND_LENGTH;
		} else if (v < 0.45) {
			return MutationType.MOVE_START;
		} else if (v < 0.6) {
			return MutationType.CHANGE_ADDITIONAL_INFO;
		} else
			return MutationType.FLIP_ACTIVE_FLAG;
	}
	
	FaultInstance changeAdditionalInfo(FaultInstance input) {
		Fault f = input.getFault();
		FaultInstance output = input;
		if (f.getName().contains("SPEEDFAULT")) {
			double newSpeed = MIN_SPEED_VALUE + r.nextDouble() * (MAX_SPEED_VALUE - MIN_SPEED_VALUE);
			output.setExtraData(Double.toString(newSpeed));	
		}
		
		if (f.getName().contains("HEADINGFAULT")) {
			double newHeading = r.nextDouble() * 360.0;
			output.setExtraData(Double.toString(newHeading));	
		}
		return output;
	}
	
	double faultInstanceIntensity(FaultInstance fi) {
		// Assume the intensity is always 1 unless otherwise
		double intensity = 1.0;
		Fault f = fi.getFault();
		Optional<String> extraData_opt = fi.getExtraDataOpt();
		if (extraData_opt.isPresent()) {
			String extraData = extraData_opt.get();
			double exValue = Double.parseDouble(extraData);
			// The speed faults intensity is relative to the max value
			if (f.getName().contains("SPEEDFAULT")) {
				intensity = exValue / MAX_SPEED_VALUE;
			}
		}
		return intensity;
	}

	// set to public for testing
	public FaultInstance mutateFaultInstanceRandomly(FaultInstance input) {
		FaultInstance output = new FaultInstance(input);
		double maxTimeShift = input.getFault().getMaxTimeRange();
		MutationType mutationType = chooseMutationOption();
		System.out.println("Performing mutation on fault instance " + input.toString());
		try {
			mutationLog.write("Performing mutation on fault instance " + input.toString() + "\n");
			switch (mutationType) {
			case CONTRACT_LENGTH:
				// Type of mutation: contracting an FI - reducing its length
				double expandFactor = r.nextDouble();
				System.out.println("Contracting length: factor = " + expandFactor);
				mutationLog.write("Performing mutation on fault instance " + input.toString() + "\n");
				output.multLengthFactor(expandFactor);
				break;
			case EXPAND_LENGTH:
				// Type of mutation: expanding an FI temporally - extending its length
				double contractFactor = r.nextDouble();
				expandFactor = 1.0 / contractFactor;
				System.out.println("Expanding length: factor = " + expandFactor);
				mutationLog.write("Expanding length: factor = " + expandFactor + "\n");
				output.multLengthFactor(expandFactor);
				break;
			case MOVE_START:
				double absTimeShift = (r.nextDouble() - 0.5) * maxTimeShift * 2;
				System.out.println("Moving fault: absTimeShift = " + absTimeShift);
				mutationLog.write("Moving fault: absTimeShift = " + absTimeShift + "\n");
				output.absShiftTimes(absTimeShift);
				break;
			case CHANGE_ADDITIONAL_INFO:
				System.out.println("Change additional info: not yet implemented");
				mutationLog.write("Change additional info: not yet implemented\n");
				output = changeAdditionalInfo(input);
			case FLIP_ACTIVE_FLAG:
				System.out.println("Flipping active flag");
				mutationLog.write("Flipping active flag\n");
				output.flipActiveFlag();
			}
			System.out.println("Mutated fault = " + output.toString());
			mutationLog.write("Mutated fault = " + output.toString() + "\n");
			mutationLog.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
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
		FaultInstanceSet best = ranks.get(0).getKey();
		newPop.add(best);
		System.out.println(
				"Best chosen:=" + best.hashCode() + ", score = " + ranks.get(0).getValue() + "\n" + best.toString());
		for (int i = 0; i < MUTATIONS_OF_SELECTED_BEST; i++) {
			FaultInstanceSet mutatedBest = best.mutateWithProb(r, MUTATION_PROB,
					fi -> mutatePossiblyMultipleTimes(fi, MAX_INDIVIDUAL_MUTATIONS));
			newPop.add(mutatedBest);
		}

		for (int i = 1; i <= EXISTING_POP_TO_RETAIN; i++) {
			FaultInstanceSet existing = ranks.get(i).getKey();
			newPop.add(existing);
		}

		System.out.println("population size at end of mutation = " + newPop.size());
		return newPop;
	}

	private void iteratePopulation() {
		iteration++;
		List<Entry<FaultInstanceSet, Double>> ranks = rankPopulation();
		setPop(mutatePopulation(getPop(), ranks));
		debugPrintPopulationWithRanks(ranks);
	}

	public void advance() {
		runNoInPopulation++;
		if (runNoInPopulation >= getPop().size()) {
			runNoInPopulation = 0;
			iteratePopulation();
		}
	}

	public List<FaultInstance> specificFaults() {
		return getPop().get(runNoInPopulation).asList();
	}

	private String specificFaultsAsString() {
		List<FaultInstance> fis = specificFaults();
		String str = fis.stream().map(f -> f.toString()).collect(Collectors.joining());
		return str;
	}

	public boolean completed() {
		return (iteration > maxIterationCount);
	}

	public void countDetections(String logFileDir, ResultInfo ri) {
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
			// If there is an object on the edge of a region, there will be excess detections. 
			// Therefore, bound the amount to zero
			int missedDetections = Math.max(0, totalExpected - detections);

			ri.setField("missedDetections", missedDetections);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}

	public void logResults(String logFileDir) {
		if (logOp != null) {
			logOp.op(this, logFileDir);
		} else {
			// process the result file and obtain the results
			FaultInstanceSet currentFS = getPop().get(runNoInPopulation);
			
			ResultInfo ri = new ResultInfo();
			countDetections(logFileDir, ri);
			getPopResults().put(currentFS, ri);
			// storedResults maintains a previous results cache
			// ensure it is used rather than re-evaluating!

			getStoredResults().put(currentFS, ri);

			try {
				double score = scoreForResult(currentFS, ri);
				int totalGoalViolations = ri.getTotalGoalViolations();
				double faultCostProportion = faultCostProportion(currentFS, ri);
				// TODO: print the number of faults too
				// TODO: print the proportion of time faults are active
				evolutionLog.write(Integer.toString(iteration) + "," + Integer.toString(runNoInPopulation) + ","
						+ Double.toString(score) + ",");
				evolutionLog.write(
						Integer.toString(totalGoalViolations) + "," + Double.toString(faultCostProportion) + "\n");
				evolutionLog.flush();
				
				populationLog.write(Integer.toString(iteration) + "," + Integer.toString(runNoInPopulation) + "," 
						+ currentFS.toString() + "\n\n");
				populationLog.flush();
				
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	public void printState() {
		System.out.println("iterations = " + iteration + ",numInPop = " + runNoInPopulation);
	}

	public List<FaultInstanceSet> getPop() {
		return pop;
	}

	public void setPop(List<FaultInstanceSet> pop) {
		this.pop = pop;
	}

	public int getRunNoInPopulation() {
		return runNoInPopulation;
	}

	public Map<FaultInstanceSet, ResultInfo> getPopResults() {
		return popResults;
	}

	public void setPopResults(Map<FaultInstanceSet, ResultInfo> popResults) {
		this.popResults = popResults;
	}

	public Map<FaultInstanceSet, ResultInfo> getStoredResults() {
		return storedResults;
	}

	public void setStoredResults(Map<FaultInstanceSet, ResultInfo> storedResults) {
		this.storedResults = storedResults;
	}
}
