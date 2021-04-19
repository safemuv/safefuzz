package atlassharedclasses;

public class BehaviourVariableUpdate {
	private String vehicleName;
	private String variableName;
	private String variableValue;
	
	BehaviourVariableUpdate() {
		
	}
	
	public BehaviourVariableUpdate(String vehicleName, String variableName, String variableValue) {
		this.vehicleName = vehicleName;
		this.variableName = variableName;
		this.variableValue = variableValue;
	}
	
	public String getVariableName() {
		return variableName;
	}
	
	public String getVariableValue() {
		return variableValue;
	}
	
	public String getVehicleName() {
		return vehicleName;
	}
}
