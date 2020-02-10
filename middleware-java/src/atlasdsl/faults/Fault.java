package atlasdsl.faults;

import java.util.Optional;

import atlasdsl.*;
import atlassharedclasses.*;

public class Fault {
	private Optional<Region> affectedRegion;
	private FaultImpact impact;
	
	public Fault(FaultImpact fi) {
		this.affectedRegion = Optional.empty();
		this.impact = fi;
	}
}
