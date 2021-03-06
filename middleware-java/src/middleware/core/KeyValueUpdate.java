package middleware.core;

import java.util.Optional;

import carsspecific.moos.carsqueue.MOOSEvent;

public class KeyValueUpdate extends MOOSEvent {
	private String vehicleName;
	private String value;
	private String sourceComponent;
	private String key;
	private double time;
	private Optional<String> carsProcess;
	
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
	
	public KeyValueUpdate(String vehicleName, String updateText, Optional<String> carsProcess, double time) throws VariableInvalid {
		this.vehicleName = vehicleName;
		this.time = time;
		this.carsProcess = carsProcess;
		setExtractedKeyAndValue(updateText);
	}
	
	public KeyValueUpdate(String vehicleName, String updateText, double time) throws VariableInvalid {
		this.vehicleName = vehicleName;
		this.time = time;
		this.carsProcess = Optional.empty();
		setExtractedKeyAndValue(updateText);
	}
	
	public KeyValueUpdate(KeyValueUpdate other) {
		this.vehicleName = other.vehicleName;
		this.time = other.time;
		this.key = other.key;
		this.value = other.value;
	}

	public boolean keyMatches(String target) {
		return target.equals(key);
	}
	
	public String getKey() {
		return key;
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
	
	public boolean hasCARSProcessName() {
		return carsProcess.isPresent();
	}
	
	public String getProcessName() {
		return carsProcess.get();
	}

	public void setValue(String replaced) {
		value = replaced;
	}

	public Optional<String> getSourceComponent() {
		return carsProcess;
	}
}
