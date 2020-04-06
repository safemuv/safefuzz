package atlasdsl.faults;

import atlasdsl.Component;

public class EnergyLoss extends ComponentImpact {
	private int fixedEnergyLoss;
	private int powerDraw;
	
	public EnergyLoss(Component c, int fixedEnergyLoss, int powerDraw) {
		super(c);
		this.fixedEnergyLoss = fixedEnergyLoss;
		this.powerDraw = powerDraw;
	}
}
