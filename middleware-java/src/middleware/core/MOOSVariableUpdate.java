package middleware.core;

public class MOOSVariableUpdate extends MOOSEvent {
	private String vehicleName;
	private String value;
	private String key;
	double time;
	
	public class VariableInvalid extends Exception {
		private static final long serialVersionUID = 1L;
	}
	
	private void setExtractedKeyAndValue(String updateText) throws VariableInvalid {
		String [] contents = updateText.split("=");
		if (contents.length > 1) {
		this.key = contents[0];
		int valpos = updateText.indexOf("=");
		this.value = updateText.substring(valpos+1);
		} else {
			throw new VariableInvalid();
		}
	}
	
	public MOOSVariableUpdate(String vehicleName, String updateText, double time) throws VariableInvalid {
		this.vehicleName = vehicleName;
		this.time = time;
		setExtractedKeyAndValue(updateText);
	}

	public boolean keyMatches(String target) {
		return target.equals(key);
	}
	
	public boolean keyStartMatches(String target) {
		int len = target.length();
		if (len <= key.length()) {
			return target.equals(key.substring(0,len));
		} else return false;
	}

	public String getValue() {
		return value;
	}
	
	public String getVehicleName() {
		return vehicleName;
	}
}
