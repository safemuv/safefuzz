package atlasdsl.faults;

import java.util.Optional;
import atlassharedclasses.*;
import middleware.core.ATLASCore;

public class Fault {
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
}