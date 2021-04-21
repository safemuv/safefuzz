package atlassharedclasses;

public class VehicleStartStopCommand extends BehaviourCommand {
	private String vehicleName;
	private boolean newStatus;
	VehicleStartStopCommand() {
		
	}
	
	public VehicleStartStopCommand(String vehicleName, boolean newStatus) {
		this.vehicleName = vehicleName;
		this.newStatus = newStatus;
	}
	
	public String getVehicleName() {
		return vehicleName;
	}
	
	public boolean getNewStatus() {
		return newStatus;
	}
}
