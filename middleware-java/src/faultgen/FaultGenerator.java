package faultgen;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Comparator;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Scanner;
import atlassharedclasses.FaultInstance;
import atlasdsl.*;
import atlasdsl.faults.*;
import middleware.core.*;

public class FaultGenerator {
	
	// Custom comparator used in the queue which sorts faults by finish time
	private class ByFinishTime implements Comparator<FaultInstance> {
	    public int compare(FaultInstance st1, FaultInstance st2) {
	        return (int)(st2.getEndTime() - st1.getEndTime());
	    }
	}
	
	private class InvalidFaultFormat extends Exception {
		private static final long serialVersionUID = 1L;		
	}
	
	private class FaultNotFoundInModel extends Exception {
		private static final long serialVersionUID = 1L;
		private String faultName;
		
		private FaultNotFoundInModel(String faultName) {
			this.faultName = faultName;
		}
	}
	
	private class FaultInstanceInvalid extends Exception {
		private static final long serialVersionUID = 1L;
	}
	
	private class FaultRepeatCountInvalid extends Exception {
		private static final long serialVersionUID = 1L;
	}
	
	private Mission mission;
	private ATLASCore core;
	
	private CountHashmap<Fault> countFaults;
	
	// This holds the fault instances that have been read from the definition file, 
	// but not yet injected. Sorted in order of injection time
	private PriorityQueue<FaultInstance> scheduledFaults = new PriorityQueue<FaultInstance>();
	
	ByFinishTime compByFinishTime = new ByFinishTime();
	
	// This holds injected fault instances in order of their removal time
	private PriorityQueue<FaultInstance> injectedFaults = new PriorityQueue<FaultInstance>(compByFinishTime);
	
	public FaultGenerator(ATLASCore core, Mission mission) {
		this.core = core;
		this.mission = mission;
	}
	
	private FaultInstance decodeFaultFromString(String faultDefinition) throws InvalidFaultFormat, InvalidComponentType, FaultNotFoundInModel, FaultInstanceInvalid, FaultRepeatCountInvalid {
		String[] fields = faultDefinition.split(",");
		if (fields.length < 3) {
			throw new InvalidFaultFormat();
		} else {
			
			int faultInstanceNum = Integer.parseInt(fields[0]);
			String faultNameInModel = fields[1];
			Double startTime = Double.parseDouble(fields[2]);
			Double length = Double.parseDouble(fields[3]);
			Double endTime = startTime + length;
			
			Optional<Fault> f_o = mission.lookupFaultByName(faultNameInModel);
			if (f_o.isPresent()) {
				Fault f = f_o.get();
				countFaults.incrementCount(f);
				if ((countFaults.getCount(f)) > f.getMaxCount()) {
					throw new FaultRepeatCountInvalid();
				}
				
				FaultInstance fi = new FaultInstance(startTime, endTime, f);
				if (!fi.isValid()) {
					throw new FaultInstanceInvalid();
				} else return fi;
			} else {
				throw new FaultNotFoundInModel(faultNameInModel);
			}
		}
	}
	
	public void loadFaultsFromFile(String filename) throws FileNotFoundException {
		countFaults = new CountHashmap<Fault>();
		File f = new File(filename);
		Scanner reader = new Scanner(f);
		while (reader.hasNextLine()) {
			String faultAsString = reader.nextLine();
			FaultInstance fault;
			try {
				fault = decodeFaultFromString(faultAsString);
				scheduledFaults.add(fault);
			} catch (InvalidFaultFormat e) {
				e.printStackTrace();
			} catch (InvalidComponentType e) {
				e.printStackTrace();
			} catch (FaultNotFoundInModel e) {
				e.printStackTrace();
			} catch (FaultInstanceInvalid e) {
				e.printStackTrace();
			} catch (FaultRepeatCountInvalid e) {
				e.printStackTrace();
			}
		}
		reader.close();
	}
	
	public void injectSpeedFaultNow(double timeLength, String robotName) {
		Robot r = mission.getRobot(robotName);
		// Need to look up the MotionSource for the target robot
		Optional<MotionSource> ms_o = r.getMotionSource(); 
		if (ms_o.isPresent()) {
			MotionSource ms = ms_o.get();
			double startTime = core.getTime();
			double endTime = startTime + timeLength;
			FaultImpact fi;
			try {
				fi = new MotionFault(ms, "UP_LOITER", "speed=5.0");
				Fault f = new Fault(fi);
				FaultInstance fInstance = new FaultInstance(startTime, endTime, f);
				core.registerFault(fInstance);
			} catch (InvalidComponentType e) {
				System.out.println("Injecting fault failed - invalid component type not a MotionSource");
				e.printStackTrace();
			}
		} else {
			System.out.println("Injecting fault failed - no motion source for " + robotName);
		}
	}
	
	public void pollFaultsNow() {
		double time = core.getTime();
		FaultInstance nextFault = scheduledFaults.peek();
		if (nextFault != null && nextFault.isReady(time)) {
			FaultInstance next = scheduledFaults.remove();
			injectedFaults.add(next);
			core.registerFault(next);
		}
		
		FaultInstance nextToFinish = injectedFaults.peek();
		if (nextToFinish != null && nextToFinish.isFinished(time)) {
			FaultInstance next = injectedFaults.remove();
			core.completeFault(next);
		}	
	}
	
	private void clearFaults() {
		scheduledFaults.clear();
	}
}
