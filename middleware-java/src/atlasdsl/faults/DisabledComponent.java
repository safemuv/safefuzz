package atlasdsl.faults;

import atlasdsl.Component;
import middleware.core.ATLASCore;

public class DisabledComponent extends ComponentImpact {
	public DisabledComponent(Component c) {
		super(c);
	}

	public void immediateEffects(ATLASCore core) {
		
	}

	public void completionEffects(ATLASCore core) {
	
	}
}
