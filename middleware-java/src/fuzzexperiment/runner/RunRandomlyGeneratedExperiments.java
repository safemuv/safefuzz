package fuzzexperiment.runner;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import atlasdsl.Mission;
import fuzzexperiment.runner.metrics.Metric;
import fuzzexperiment.runner.metrics.OfflineMetric;
import fuzzingengine.exptgenerator.*;

public class RunRandomlyGeneratedExperiments extends ExptParams {
	private String resFileName;
	int count = 0;
	int countLimit;
	private Mission mission;
	private String fuzzCSVBaseName;
	private	FuzzingExperimentGenerator g;
	private int runNumFixed;
	
	private String getCurrentFilename() {
		return fuzzCSVBaseName + "-" + count + ".csv";
	}
	
	private void newGeneratedFile() {
		g.generateExperiment(Optional.of(getCurrentFilename()));
	}
	
	public RunRandomlyGeneratedExperiments(String resFileName, Mission mission, String fuzzCSVBaseName, int countLimit, FuzzingTimeSpecificationGenerator tgen, int runNumFixed) {
		this.resFileName = resFileName;
		this.mission = mission;
		this.countLimit = countLimit;
		this.fuzzCSVBaseName = fuzzCSVBaseName;
		this.runNumFixed = runNumFixed;
		g = new FuzzingExperimentGenerator(tgen, mission);
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
		count++;
		newGeneratedFile();
	}

	@Override
	public void printStateAfter() throws IOException {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void printFinal(List<OfflineMetric> ms) {
		// TODO Auto-generated method stub
		
	}

	protected int getRunNum() {
		return runNumFixed;
	}
}
