package ciexperiment.systematic;

import java.util.HashMap;
import java.util.List;
import exptrunner.jmetal.FuzzingSelectionsSolution;
import fuzzingengine.FuzzingSelectionRecord;

public abstract class ExptParams {
	protected double runtime;
	
	protected HashMap<FuzzingSelectionsSolution,Double> solutionLog = new HashMap<FuzzingSelectionsSolution,Double>();

	public double getTimeLimit() {
		return runtime;
	}
	
	public ExptParams(double runtime) {
		this.runtime = runtime;
	}

	public abstract boolean completed();
	public abstract void printState();
	public abstract void advance();
	public abstract void logResults(String string);
	
	public HashMap<FuzzingSelectionsSolution,Double> returnResultsInfo() {
		return solutionLog;
	}
	
	public abstract List<FuzzingSelectionRecord> specificFaults();
}