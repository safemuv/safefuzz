package atlasdsl.faults;

import java.util.Optional;

import atlassharedclasses.*;
import middleware.core.ATLASCore;

public class Fault {
	private Optional<Region> affectedRegion;
	private FaultImpact impact;
	private FaultTimeProperties timeProperties;
	
	public Fault(FaultImpact fi) {
		this.affectedRegion = Optional.empty();
		this.impact = fi;
	}
	
	public Fault(FaultImpact fi, Optional<Region> r, FaultTimeProperties timeProps) {
		this.affectedRegion = r;
		this.impact = fi;
		this.timeProperties = timeProps;
	}

	public Object applyFaultToData(Object orig) {
		return impact.applyImpact(orig);
	}

	public void immediateEffects(ATLASCore core) {
		impact.immediateEffects(core);
		System.out.println("Fault.immediateEffects");
	}
}
