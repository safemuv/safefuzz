package atlasdsl.faults;

import java.util.Optional;

import atlasdsl.Component;
import middleware.core.ATLASCore;

public class DisabledComponent extends ComponentImpact {
	public DisabledComponent(Component c) {
		super(c);
	}

	public void immediateEffects(ATLASCore core, Optional<String> additionalData) {
		
	}

	public void completionEffects(ATLASCore core, Optional<String> additionalData) {
	
	}
}
