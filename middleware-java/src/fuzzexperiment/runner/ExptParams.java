package fuzzexperiment.runner;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import exptrunner.jmetal.FuzzingSelectionsSolution;
import fuzzexperiment.runner.metrics.Metric;

public abstract class ExptParams {
	
	protected HashMap<FuzzingSelectionsSolution,Double> solutionLog = new HashMap<FuzzingSelectionsSolution,Double>();

	public abstract boolean completed();
	public abstract void printState() throws IOException;
	public abstract void printStateAfter() throws IOException;
	public abstract void advance();
	public abstract void advance(Map<Metric, Double> res);
	
	public HashMap<FuzzingSelectionsSolution,Double> returnResultsInfo() {
		return solutionLog;
	}
	
	protected abstract Optional<String> getNextFuzzingCSVFileName();
}