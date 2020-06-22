package exptrunner;

import java.util.List;
import atlassharedclasses.FaultInstance;

public abstract class ExptParams {
	private double runtime = 1200.0;

	public double getTimeLimit() {
		return runtime;
	}

	public abstract boolean completed();
	public abstract void printState();
	public abstract void advance();
	public abstract void logResults(String string);
	public abstract List<FaultInstance> specificFaults();
}