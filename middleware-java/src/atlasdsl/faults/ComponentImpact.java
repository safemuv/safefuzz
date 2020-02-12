package atlasdsl.faults;

import atlasdsl.*;

public class ComponentImpact extends FaultImpact {
	private Component affectedComponent;

	public Object applyImpact(Object orig) {
		return orig;
	}
}
