package moosmapping;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public abstract class MOOSElement {
	// Hash map of string properties
	protected HashMap<String, Object> properties = new HashMap<String,Object>();  
	
	protected void writeProperties(FileWriter stream, String sep, String lineDelim) throws IOException {
		for (Map.Entry<String,Object> entry : properties.entrySet()) {
			String name = entry.getKey();
			Object value = entry.getValue();
			stream.write(name + sep + value.toString() + lineDelim);
		}
	}
	
	protected void writePropertiesDefault(FileWriter stream) throws IOException {
		writeProperties(stream, " = ", "\n");
	}
	
	public void setProperty(String key, Object val) {
		properties.put(key, val);
	}
	
	public Object getProperty(String key) {
		return properties.get(key);
	}
}
