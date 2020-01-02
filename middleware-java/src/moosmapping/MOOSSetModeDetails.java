package moosmapping;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;

public class MOOSSetModeDetails extends MOOSElement {
	private String modeName;
	private String extraTag = "";
	
	MOOSSetModeDetails(String modeName, String extraTag) {
		this.modeName = modeName;
		this.extraTag = extraTag;
	}
	
	MOOSSetModeDetails(String modeName) {
		this.modeName = modeName;
	}
	
	protected void generate(FileWriter stream) throws IOException {
		stream.write("set MODE = " + modeName + " {\n");
		for (Map.Entry<String,Object> entry : properties.entrySet()) {
			String name = entry.getKey();
			Object value = entry.getValue();
			stream.write(name + " = " + value.toString() + "\n");
		}
		stream.write("} " + extraTag + "\n\n");
	}
}
