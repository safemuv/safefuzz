package fuzzexperiment.runner;

import java.util.HashMap;
import java.util.Optional;
import exptrunner.jmetal.FuzzingSelectionsSolution;

public abstract class ExptParams {
	
	protected HashMap<FuzzingSelectionsSolution,Double> solutionLog = new HashMap<FuzzingSelectionsSolution,Double>();

	public abstract boolean completed();
	public abstract void printState();
	public abstract void advance();
	public abstract void logResults(String string, String modelFile);
	
	public HashMap<FuzzingSelectionsSolution,Double> returnResultsInfo() {
		return solutionLog;
	}
	
	public abstract Optional<String> getNextFileName();
}