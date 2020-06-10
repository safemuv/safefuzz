package atlassharedclasses;

import java.util.HashMap;
import java.util.LinkedHashMap;

import atlasdsl.*;

public class SensorDetection {
	private Message msg;
	private SensorType type;
	private HashMap<String,Object> fields = new LinkedHashMap<String,Object>();

	public SensorDetection(Message m, SensorType t) {
		this.msg = m;
		this.type = t;
	}
	
	public void setField(String key, Object v) {
		fields.put(key,v);
	}
	
	public Object getField(String key) {
		return fields.get(key);
	}
	
	SensorDetection() {
		
	}
	
	public Message getMessage() {
		return msg;
	}
	
	public SensorType getSensorType() {
		return type;
	}
	
	public void setSensorType(SensorType t) {
		this.type = t;
	}
	
	public HashMap<String,Object> getFields() {
		return fields;
	}
	
	public String toString() {
		return type.toString() + "-" + fields.toString();
	}
}
