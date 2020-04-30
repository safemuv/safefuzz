package atlassharedclasses;

public class StartVehicle extends BehaviourCommand {
	private String vehicleName;
	StartVehicle() {
		
	}
	
	public StartVehicle(String vehicleName) {
		this.vehicleName = vehicleName;
	}
	
	public String getVehicleName() {
		return vehicleName;
	}
}
