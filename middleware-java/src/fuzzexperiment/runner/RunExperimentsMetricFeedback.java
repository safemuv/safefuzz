package fuzzexperiment.runner;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import atlasdsl.Mission;
import fuzzexperiment.runner.metrics.Metric;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.exptgenerator.FuzzingExperimentModifier;

public class RunExperimentsMetricFeedback extends ExptParams {

	private String fuzzCSVBaseName;
	private String resFileName;
	int count = 1;
	int countLimit;
	private Mission mission;
	List<Metric> metrics;
	
	private int populationLimit;

	private FuzzingExperimentModifier g;

	public FuzzingPopulation pop = new FuzzingPopulation(populationLimit);
	private List<FuzzingSelectionRecord> currentFuzzingSels;

	private String getCurrentFilename() {
		return fuzzCSVBaseName + "-" + String.format("%03d", count) + ".csv";
	}

	private void newRandomFile() {
		currentFuzzingSels = g.generateExperiment(Optional.of(getCurrentFilename()));
	}

	public RunExperimentsMetricFeedback(String resFileName, Mission mission, String fuzzCSVBaseName, int countLimit, int populationLimit) {
		this.resFileName = resFileName;
		this.populationLimit = populationLimit;
		this.mission = mission;
		this.countLimit = countLimit;
		this.fuzzCSVBaseName = fuzzCSVBaseName;
		g = new FuzzingExperimentModifier(mission);
		newRandomFile();
		
		metrics = new ArrayList<Metric>(mission.getAllMetrics());
	}

	public boolean completed() {
		return (count >= countLimit);
	}

	public void printState() {
		System.out.println("RunExperimentsMetricFeedback: Evaluating entry " + getCurrentFilename());
	}

	public void advance() {
		count++;
		newRandomFile();
	}

	public Optional<String> getNextFuzzingCSVFileName() {
		return Optional.of(getCurrentFilename());
	}

	public void advance(Map<Metric, Double> res) {
		pop.pushToPopulation(new FuzzingExptResult(currentFuzzingSels, getCurrentFilename(), res));
		count++;
		System.out.print("Population size = " + pop.currentSize() + "(limit " + populationLimit + ")");

		if (pop.currentSize() < populationLimit) {
			System.out.println("- Generating new fuzzing experiment");
			newRandomFile();		
		} else {
			Optional<FuzzingExptResult> startingPoint_o = pop.pickPopulationElementToExplore();
			if (startingPoint_o.isPresent()) {
				FuzzingExptResult startingPoint = startingPoint_o.get();
				System.out.println("- Generating mutated experiment based upon " + startingPoint);
				List<FuzzingSelectionRecord> sels = startingPoint.getFuzzingSpec();
				System.out.println("startingPoint = " + startingPoint);
				System.out.println("\nSelected fuzzing:\n" + sels);
				currentFuzzingSels = g.generateExperimentBasedUpon(getCurrentFilename(), sels, res);
			}
		}
	}
}
