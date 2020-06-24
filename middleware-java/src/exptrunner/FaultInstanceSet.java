package exptrunner;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
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
	
	public FaultInstanceSet(CreationOperation op, int count) {
		Set<FaultInstance> fs = new HashSet<FaultInstance>();
		
	}
	
	public int getCount() {
		return fs.size();
	}
	
	public FaultInstanceSet mutateBy(int countToMutate, MutationOperation mut) {
		// TODO: implement mutation 
		return new FaultInstanceSet(this.fs);
	}
	
	public FaultInstanceSet mutateWithProb(double prob, MutationOperation mut) {
		// TODO: implement mutation 
		return new FaultInstanceSet(this.fs);
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
}
