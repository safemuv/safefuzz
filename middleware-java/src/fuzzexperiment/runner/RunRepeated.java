package fuzzexperiment.runner;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import fuzzexperiment.runner.metrics.Metric;
import fuzzexperiment.runner.metrics.OfflineMetric;

public class RunRepeated extends ExptParams {
	private int runNum = 0;
	private int limitCount;
	private String fixedFilename;
	private boolean startLaunchers;
	
	public RunRepeated(String fixedFilename, int limitCount, int runNumStart, boolean startLaunchers) {
		this.runNum = runNumStart;
		this.limitCount = limitCount + runNumStart;
		this.fixedFilename = fixedFilename;
		this.startLaunchers = startLaunchers;
	}

	public boolean completed() {
		return (runNum >= limitCount);
	}

	public void printState() {
		System.out.println("Run num = " + runNum);
	}

	public void advance() {
		runNum++;
	}

	public Optional<String> getNextFuzzingCSVFileName() {
		return Optional.of(fixedFilename);
	}

	public void advance(Map<Metric, Double> res) {
		runNum++;
		
	}

	public void printStateAfter() throws IOException {
		// TODO Auto-generated method stub
		
	}

	protected void printFinal(List<OfflineMetric> ms) {
		// TODO Auto-generated method stub
		
	}

	protected int getRunNum() {
		return runNum;
	}

}
