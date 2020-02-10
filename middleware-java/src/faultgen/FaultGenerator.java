package faultgen;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.regex.Pattern;

import atlassharedclasses.ATLASObjectMapper;
import atlassharedclasses.FaultInstance;
import atlasdsl.*;
import atlasdsl.faults.*;
import middleware.core.*;

public class FaultGenerator {
	
	private class InvalidFaultFormat extends Exception {
		private static final long serialVersionUID = 1L;		
	}
	
	private Mission mission;
	private ATLASCore core;
	
	// This holds the faults that have been read from the definition file, 
	// but not yet injected. Sorted in order of time
	private PriorityQueue<FaultInstance> scheduledFaults = new PriorityQueue<FaultInstance>();
	
	public FaultGenerator(ATLASCore core, Mission mission) {
		this.core = core;
		this.mission = mission;
	}
	
	private FaultInstance decodeFaultFromString(String faultDefinition) throws InvalidFaultFormat {
		String[] fields = faultDefinition.split(",");
		if (fields.length < 3) {
			throw new InvalidFaultFormat();
		} else {
			Double startTime = Double.parseDouble(fields[0]);
			Double endTime = Double.parseDouble(fields[1]);
			String faultImpactStr = fields[2];
			FaultImpact fi;
			// TODO: better solution here
			if (faultImpactStr == "MutateMessage") {
				fi = new MutateMessage();
				Fault f = new Fault(fi);
				return new FaultInstance(startTime, endTime, f);
			}	
		}
		throw new InvalidFaultFormat();
	}
	
	public void loadFaultsFromFile(String filename) throws FileNotFoundException {
		File f = new File(filename);
		Scanner reader = new Scanner(f);
		// Format: start time, end time, vehicle, fault impact description
		while (reader.hasNextLine()) {
			String faultAsString = reader.nextLine();
			FaultInstance fault;
			try {
				fault = decodeFaultFromString(faultAsString);
				scheduledFaults.add(fault);
			} catch (InvalidFaultFormat e) {
				e.printStackTrace();
			}
		}
		reader.close();
	}
	
	public void pollFaultsNow() {
		double time = core.getTime();
		FaultInstance nextFault = scheduledFaults.peek();
		if (nextFault != null && nextFault.isReady(time)) {
			FaultInstance next = scheduledFaults.remove();
			// TODO: Core should receive FaultEvents, not the actual 
			// faults themselves?
			core.registerFault(next);
		}
	}
	
	private void clearFaults() {
		scheduledFaults.clear();
	}
}
