package faultgen;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.PriorityQueue;
import java.util.Scanner;
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
			double length = Double.parseDouble(fields[1]);
			Double endTime = length + startTime;
			
			String vehicleName = fields[2];
			String faultImpactStr = fields[3].toUpperCase();
			FaultImpact fi;
			// TODO: better solution here
			// need to specify what the message mutates
			if (faultImpactStr == "MUTATEMESSAGE") {
				fi = new MutateMessage();
				Fault f = new Fault(fi);
				return new FaultInstance(startTime, endTime, f);
			}
			
			if (faultImpactStr == "OVERSPEED") {
				Double speed = Double.parseDouble(fields[4]);
				Robot r = mission.getRobot(vehicleName);
				fi = new MotionFault(r, "UP_LOITER", "speed=5.0");
				Fault f = new Fault(fi);
				FaultInstance fInstance = new FaultInstance(startTime, endTime, f);
				return fInstance;
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
	
	public void injectSpeedFaultNow(double timeLength, String robotName) {
		Robot r = mission.getRobot(robotName);
		
		double startTime = core.getTime();
		double endTime = startTime + timeLength;
		FaultImpact fi = new MotionFault(r, "UP_LOITER", "speed=5.0");
		Fault f = new Fault(fi);
		FaultInstance fInstance = new FaultInstance(startTime, endTime, f);
		core.registerFault(fInstance);
		
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
