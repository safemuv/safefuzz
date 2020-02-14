package atlasdsl.faults;

import atlasdsl.*;
import middleware.core.ATLASCore;

public class ComponentImpact extends FaultImpact {
	private Component affectedComponent;
	private Component changeProperty;

	public Object applyImpact(Object orig) {
		return orig;
	}

	public void immediateEffects(ATLASCore core) {
		
	}
}
