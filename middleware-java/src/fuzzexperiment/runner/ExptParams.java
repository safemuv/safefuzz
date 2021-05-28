package fuzzexperiment.runner;

import java.util.HashMap;
import java.util.Optional;
import exptrunner.jmetal.FuzzingSelectionsSolution;

public abstract class ExptParams {
	
	protected HashMap<FuzzingSelectionsSolution,Double> solutionLog = new HashMap<FuzzingSelectionsSolution,Double>();

	public abstract boolean completed();
	public abstract void printState();
	public abstract void advance();
	
	public HashMap<FuzzingSelectionsSolution,Double> returnResultsInfo() {
		return solutionLog;
	}
	
	protected abstract Optional<String> getNextFuzzingCSVFileName();
}