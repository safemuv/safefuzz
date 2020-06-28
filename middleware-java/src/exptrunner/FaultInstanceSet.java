package exptrunner;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import static java.util.stream.Collectors.*;

import atlassharedclasses.*;
import exptrunner.operations.*;

public class FaultInstanceSet {
	private Set<FaultInstance> fs = new HashSet<FaultInstance>();
	
	public FaultInstanceSet(List<FaultInstance> fis) {
		this.fs = new HashSet<FaultInstance>(fis);
	}
	
	public FaultInstanceSet(Set<FaultInstance> fis) {
		this.fs = fis;
	}
	
	public FaultInstanceSet(CreationOperation creationOp, int count) {
		for (int i = 0; i < count; i++) 
			fs.add(creationOp.op(i));
	}
	
	public int getCount() {
		return fs.size();
	}
	
	public Set<FaultInstance> getFaultInstances() {
		return fs;
	}
	
	//public FaultInstanceSet mutateBy(Random r, int countToMutate, MutationOperation mut) {
		// TODO: implement mutation 
		//for (FaultInstance fi : fs) {
			//
//		}
//		return new FaultInstanceSet(this.fs);
	//}
	
	public FaultInstanceSet mutateWithProb(Random r, double prob, MutationOperation mut) {
		Set<FaultInstance> outputFaultSet = new HashSet<FaultInstance>();
		for (FaultInstance origFi : this.fs) {
			FaultInstance copy = new FaultInstance(origFi);
			double v = r.nextDouble();
			if (v < prob) {
				outputFaultSet.add(mut.op(copy));
			} else {
				outputFaultSet.add(copy);
			}
		}
		return new FaultInstanceSet(outputFaultSet);
	}
	
	public List<FaultInstance> asList() {
		return fs.stream().collect(toCollection(ArrayList::new));
	}

	public double totalFaultTimeLength() {
		double total = 0.0;
		for (FaultInstance fi : fs) {
			total += (fi.getEndTime() - fi.getStartTime());
		}
		return total;
	}
	
	public double totalActiveFaultTimeLength() {
		double total = 0.0;
		for (FaultInstance fi : fs) {
			if (fi.isActive()) {
				total += (fi.getEndTime() - fi.getStartTime());
			}
		}
		return total;
	}
	
	public String toString() {
		String output = "HASHCODE:" + this.hashCode() + " - " + this.getCount() + " faults\n";
		for (FaultInstance fi : fs) {
			output = output + fi.toString() + "\n";
		}
		return output;
	}
	
	public boolean _hasFaultInRange(double trigPoint, double minSize, String faultName) {
		boolean isPresent = false;
		for (FaultInstance fi : fs) {
			String faultNameForInstance = fi.getFault().getName();
			//System.out.println("faultNameForInstance = " + faultNameForInstance);
			boolean faultNameMatch = faultNameForInstance.equals(faultName);
			System.out.println("faultNameMatch = " + faultNameMatch);
			double length = fi.getLength();
			if ((trigPoint > fi.getStartTime()) && (trigPoint < fi.getEndTime()) && (length < minSize) && faultNameMatch) {
				isPresent = true;
			}
		}
		return isPresent;
	} 
}
