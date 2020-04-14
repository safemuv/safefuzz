package atlasdsl.faults;

import atlasdsl.*;
import middleware.core.ATLASCore;

public abstract class ComponentImpact extends FaultImpact {
	protected Component affectedComponent;

	public ComponentImpact(Component c) {
		this.affectedComponent = c;
	}

	public Object applyImpact(Object orig) {
		return orig;
	}

	public abstract void immediateEffects(ATLASCore core);
	public abstract void completionEffects(ATLASCore core);
}
