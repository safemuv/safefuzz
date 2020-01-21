package middleware.core;

public class MOOSVariableUpdate extends MOOSEvent {
	private String vehicleName;
	private String value;
	private String key;
	double time;
	
	private void setExtractedKeyAndValue(String updateText) {
		// TODO: split at first equals
		this.key = "K";
		this.value = "V";
	}
	
	public MOOSVariableUpdate(String vehicleName, String updateText, double time) {
		this.vehicleName = vehicleName;
		this.time = time;
		setExtractedKeyAndValue(updateText);
	}

	public boolean matchesKeyStart(String target) {
		// TODO: if the given string is contained in the start of the key
		return false;
	}
	
	public boolean matchesKey(String target) {
		return key.equals(target);
	}

	public String getValue() {
		return value;
	}
}
