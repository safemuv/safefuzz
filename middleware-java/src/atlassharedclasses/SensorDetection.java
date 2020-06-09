package atlassharedclasses;

import java.util.HashMap;
import java.util.LinkedHashMap;

import atlasdsl.*;

public class SensorDetection {
	private Message m;
	private SensorType t;
	private HashMap<String,Object> fields = new LinkedHashMap<String,Object>();

	public SensorDetection(Message m, SensorType t) {
		this.m = m;
		this.t = t;
	}
	
	public void setField(String key, Object v) {
		fields.put(key,t);
	}
	
	public Object getField(String key) {
		return fields.get(key);
	}
	
	SensorDetection() {
		
	}
	
	public Message getMessage() {
		return m;
	}
	
	public SensorType getSensorType() {
		return t;
	}
	
	public HashMap<String,Object> getFields() {
		return fields;
	}
}
