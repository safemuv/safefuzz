package atlasdsl.faults;

import java.util.List;
import java.util.Optional;

import atlasdsl.*;
import atlassharedclasses.*;
import middleware.core.ATLASCore;

public class Fault {
	private Optional<Region> affectedRegion;
	private FaultImpact impact;
	
	public Fault(FaultImpact fi) {
		this.affectedRegion = Optional.empty();
		this.impact = fi;
	}

	public Object applyFaultToData(Object orig) {
		return impact.applyImpact(orig);
	}

	public void immediateEffects(ATLASCore core) {
		impact.immediateEffects(core);
		System.out.println("Fault.immediateEffects");
	}
}
