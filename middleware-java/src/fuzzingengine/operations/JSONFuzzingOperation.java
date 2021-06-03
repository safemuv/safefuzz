package fuzzingengine.operations;

import java.io.StringReader;

import javax.json.*;

public abstract class JSONFuzzingOperation extends ValueFuzzingOperation {
	private final boolean DEBUG_JSON_FUZZING = true;
	
	public String fuzzTransformString(String input) {
		JsonReader jsonReader = Json.createReader(new StringReader(input));
		JsonStructure str = jsonReader.read();
		jsonReader.close();
		if (DEBUG_JSON_FUZZING) {
			System.out.println(this.getClass().getName() + ": JSONFuzzingOperation.fuzzTransformString received message JSON " + str.toString());
		}
		String output = fuzzTransformMessage(str).toString();
		if (DEBUG_JSON_FUZZING) {
			System.out.println(this.getClass().getName() +  ": JSONFuzzingOperation.fuzzTransformMessage modified it to " + output);
		}
		return output;
	}
	
	public abstract JsonStructure fuzzTransformMessage(JsonStructure js);
}
