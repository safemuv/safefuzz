package atlassharedclasses;

public class EnergyUpdate extends SensorInfo {
	private double energyValue;
	private String robotName;
	
	// There has to be a default constructor to allow the class to be 
	// serialised.
	public EnergyUpdate() {
		
	}
	
	public EnergyUpdate(double energyValue, String robotName) {
		this.energyValue = energyValue;
		this.robotName = robotName;
	}
	
	public String getRobotName() {
		return robotName;
	}

	public double getEnergyValue() {
		return energyValue;
	}
}
