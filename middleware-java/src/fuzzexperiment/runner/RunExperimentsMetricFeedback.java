package fuzzexperiment.runner;

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
	int count = 0;
	int countLimit;
	private Mission mission;
	
	int populationLimit = 10;

	private	FuzzingExperimentModifier g;
	
	public FuzzingPopulation pop = new FuzzingPopulation(populationLimit);
	private List<FuzzingSelectionRecord> currentFuzzingSels;

	private String getCurrentFilename() {
		return fuzzCSVBaseName + "-" + count + ".csv";
	}
	
	private void newGeneratedFile() {
		g.generateExperiment(Optional.of(getCurrentFilename()));
	}
	
	public RunExperimentsMetricFeedback(String resFileName, Mission mission, String fuzzCSVBaseName, int countLimit) {
		this.resFileName = resFileName;
		this.mission = mission;
		this.countLimit = countLimit;
		this.fuzzCSVBaseName = fuzzCSVBaseName;
		g = new FuzzingExperimentModifier(mission);
		newGeneratedFile();
	}

	public boolean completed() {
		return (count >= countLimit);
	}

	public void printState() {
		System.out.println("Evaluating entry " + getCurrentFilename());
	}

	public void advance() {
		count++;
		newGeneratedFile();
	}

	public Optional<String> getNextFuzzingCSVFileName() {
		return Optional.of(getCurrentFilename());
	}

	public void advance(Map<Metric, Double> res) {
		pop.pushToPopulation(new FuzzingExptResult(currentFuzzingSels, getCurrentFilename(), res));
		count++;
		Optional<FuzzingExptResult> startingPoint_o = pop.pickPopulationElementToExplore();
		if (startingPoint_o.isPresent()) {
			String startingFilename = startingPoint_o.get().getFilename();
			currentFuzzingSels = g.generateExperimentBasedUpon(getCurrentFilename(), Optional.of(startingFilename), res);
		} else {
			currentFuzzingSels = g.generateExperiment(Optional.of(getCurrentFilename()));
		}
	}
}
