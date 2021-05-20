package fuzzingengine.operations;

import java.io.StringReader;

import javax.json.Json;
import javax.json.JsonObject;
import javax.json.JsonReader;

public abstract class JSONFuzzingOperation extends ValueFuzzingOperation {
	private final boolean DEBUG_JSON_FUZZING = true;
	
	public String fuzzTransformString(String input) {
		JsonReader jsonReader = Json.createReader(new StringReader(input));
		JsonObject obj = jsonReader.readObject();
		jsonReader.close();
		if (DEBUG_JSON_FUZZING) {
			System.out.println(this.getClass().getName() + ": JSONFuzzingOperation.fuzzTransformString received message JSON " + obj.toString());
		}
		String output = fuzzTransformMessage(obj).toString();
		if (DEBUG_JSON_FUZZING) {
			System.out.println(this.getClass().getName() +  ": JSONFuzzingOperation.fuzzTransformMessage modified it to " + output);
		}
		return output;
	}
	
	public abstract JsonObject fuzzTransformMessage(JsonObject msg);
}
