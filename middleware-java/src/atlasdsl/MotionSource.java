package atlasdsl;

public class MotionSource extends Subcomponent {
	private double energyPerDistance;
	
	public MotionSource(double energyPerDistance) {
		this.energyPerDistance = energyPerDistance;
	}
	
	public double getEnergyPerDistance() {
		return energyPerDistance;
	}
}
