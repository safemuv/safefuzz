package atlasdsl.faults;

import atlasdsl.*;
import middleware.core.ATLASCore;

public class ComponentImpact extends FaultImpact {
	protected Component affectedComponent;

	public ComponentImpact(Component c) {
		this.affectedComponent = c;
	}

	public Object applyImpact(Object orig) {
		return orig;
	}

	public void immediateEffects(ATLASCore core) {
		
	}

	public void completionEffects(ATLASCore core) {
		
	}
}
