package atlasdsl.faults;

import middleware.core.ATLASCore;

public abstract class FaultImpact {
	public abstract Object applyImpact(Object orig);
	public abstract void immediateEffects(ATLASCore core);
}
