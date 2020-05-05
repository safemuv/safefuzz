package exptrunner;

import java.util.List;

import atlassharedclasses.FaultInstance;

public class SingleFaultCoverageExpt extends ExptParams {
	private double minLength;
	private double maxLength;
	
	// The time range to be swept 
	private double timeStart;
	private double timeEnd;
	
	// The fault number to use
	private int numberOfFaultInModel;
	
	// The current fault ID
	private int currentFault;
	
	private boolean completed = false;
	
	public SingleFaultCoverageExpt(double timeStart, double timeEnd, double maxLength, double minLength) {
		this.timeStart = timeStart;
		this.timeEnd = timeEnd;
		this.maxLength = maxLength;
		this.minLength = minLength;
		this.completed = false;
	}

	public void advance() {
		
	}

	public List<FaultInstance> specificFaults() {
		// Generate a single fault here
		return null;
	}

	// TODO: for now, assume completed after once!
	public boolean completed() {
		boolean res = completed;
		completed = true;
		return res;
		
	}
}
