package exptrunner;

import java.util.List;

import atlassharedclasses.FaultInstance;

public abstract class ExptParams {
	private double runtime = 1000.0;

	public double getTimeLimit() {
		return runtime;
	}

	public abstract boolean completed();
	public abstract void advance();


	public void processGoalResults(String string) {
		// TODO: process the goal results file log
		System.out.println("Need to process the result file here!");
	}
	
	public abstract List<FaultInstance> specificFaults();
}
