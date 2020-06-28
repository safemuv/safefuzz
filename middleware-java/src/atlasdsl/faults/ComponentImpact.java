package atlasdsl.faults;

import java.util.Optional;

import atlasdsl.*;
import middleware.core.ATLASCore;

public abstract class ComponentImpact extends FaultImpact {
	protected Component affectedComponent;

	public ComponentImpact(Component c) {
		this.affectedComponent = c;
	}

	public Object applyImpact(Object orig, Optional<String> additionalData) {
		return orig;
	}

	public abstract void immediateEffects(ATLASCore core, Optional<String> additionalData);
	public abstract void completionEffects(ATLASCore core, Optional<String> additionalData);
}
