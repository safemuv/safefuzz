package atlasdsl;

import java.util.Optional;
import middleware.core.ATLASCore;

public class ReturnOnLowEnergy extends GoalAction {
	private double energyThreshold;
	
	public ReturnOnLowEnergy(double energyThreshold) {
		this.energyThreshold = energyThreshold;
	}
	
	protected Optional<GoalResult> test(Mission mission, GoalParticipants participants) {
		return Optional.empty();
	}

	protected void setup(ATLASCore core, Mission mission, Goal g) throws GoalActionSetupFailure {

	}
	
	public double getEnergyThreshold() {
		return energyThreshold;
	}
}
