package atlasdsl.faults;

import java.util.Optional;

import middleware.core.ATLASCore;

public abstract class FaultImpact {
	public abstract Object applyImpact(Object orig, Optional<String> additionalData);
	public abstract void immediateEffects(ATLASCore core, Optional<String> additionalData);
	public abstract void completionEffects(ATLASCore core, Optional<String> additionalData);
}
