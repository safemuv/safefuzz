package atlasdsl.faults;

import java.util.Optional;

import atlasdsl.Battery;
import atlasdsl.Component;
import atlasdsl.InvalidComponentType;
import middleware.core.ATLASCore;

public class EnergyLoss extends ComponentImpact {
	private int fixedEnergyLoss;
	
	public EnergyLoss(Component c, int fixedEnergyLoss, int powerDraw) throws InvalidComponentType {
		// Verify here that the component is a Battery
		// For now, EnergyLoss faults are only considering sudden depletion of a battery
		super(c);
		if (c instanceof Battery) {
			this.fixedEnergyLoss = fixedEnergyLoss;
		} else {
			throw new InvalidComponentType(c, c.getClass(), Battery.class);
		}	
	}
	
	public void immediateEffects(ATLASCore core, Optional<String> additionalData) {
		Battery b = (Battery)affectedComponent;
		b.depleteEnergy(fixedEnergyLoss);
	}

	public void completionEffects(ATLASCore core, Optional<String> additionalData) {
		
	}
}
