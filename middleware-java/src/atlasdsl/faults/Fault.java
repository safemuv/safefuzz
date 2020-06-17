package atlasdsl.faults;

import java.util.Optional;
import java.util.Random;

import atlassharedclasses.*;
import middleware.core.ATLASCore;

public class Fault {
	Random random = new Random();
	
	private Optional<Region> affectedRegion;
	private FaultImpact impact;
	private FaultTimeProperties timeProperties;
	private String name;
	
	public Fault(FaultImpact fi) {
		this.affectedRegion = Optional.empty();
		this.impact = fi;
	}
	
	public Fault(String name, FaultImpact fi, Optional<Region> r, FaultTimeProperties timeProps) {
		this.name = name;
		this.affectedRegion = r;
		this.impact = fi;
		this.timeProperties = timeProps;
	}

	public Object applyFaultToData(Object orig) {
		return impact.applyImpact(orig);
	}

	public void immediateEffects(ATLASCore core) {
		impact.immediateEffects(core);
	}
	
	public void completionEffects(ATLASCore core) {
		impact.completionEffects(core);
	}
	
	public FaultImpact getImpact() {
		return impact;
	}
	
	public String getName() {
		return name;
	}
	
	public FaultTimeProperties getTimeProperties() {
		return timeProperties;
	}

	public int getMaxCount() {
		return timeProperties.getMaxRepeatCount();
	}

	public FaultInstance randomWithin(FaultCreationTypes randomUniform) {		
		double earliestStartTime = timeProperties.getEarliestStart();
		double latestEndTime = timeProperties.getLatestEnd();
		double maxLen = timeProperties.getMaxFaultLength();
		
		double offsetFromEarliest = random.nextDouble() * (latestEndTime - earliestStartTime);
		double startTime = earliestStartTime + offsetFromEarliest;
		double limitedLen = (latestEndTime - startTime);
		System.out.println("limitedLen = " + limitedLen + ",maxLen=" + maxLen);
				
		double length = Math.min((random.nextDouble() * maxLen), limitedLen);
		double endTime = startTime + length;
		
		FaultInstance fi = new FaultInstance(startTime, endTime, this, Optional.empty());
		return fi;
	}

	public double getEarliestStartTime() {
		return timeProperties.getEarliestStart();
	}
	
	public double getLatestEndTime() {
		return timeProperties.getLatestEnd();
	}
}