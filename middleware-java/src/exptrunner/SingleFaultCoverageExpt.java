package exptrunner;

import java.util.ArrayList;
import java.util.List;

import atlasdsl.faults.*;
import atlassharedclasses.FaultInstance;

public class SingleFaultCoverageExpt extends ExptParams {
	private double minLength;
	private double maxLength;
	
	// The time range to be swept 
	private double timeStart;
	private double timeEnd;
	
	private double time;
	private double len;
	
	// The fault number to use
	private Fault fault;
	
	// The current fault ID
	private int currentFault;
	
	private boolean completed = false;
	
	public SingleFaultCoverageExpt(double timeStart, double timeEnd, double maxLength, double minLength, Fault fault) {
		this.timeStart = timeStart;
		this.time = timeStart;
		this.timeEnd = timeEnd;
		this.maxLength = maxLength;
		this.len = maxLength;
		this.minLength = minLength;
		this.completed = false;
		this.fault = fault;
	}

	public void advance() {
		time += len;
		if (time > timeEnd) {
			time = timeStart;
			len = len * 0.75;
		}
	}

	public List<FaultInstance> specificFaults() {
		List<FaultInstance> fs = new ArrayList<FaultInstance>();
		System.out.println("Generating fault instance " + time + "-" + len);
		FaultInstance fi = new FaultInstance(time, len, fault);
		fs.add(fi);
		return fs;
	}

	public boolean completed() {
		return (len < minLength); 
	}

	public void logResults(String string) {
		// Read the goal result file here - process the given goals
		// Write it out to a common result file - with the fault info
	}
}
