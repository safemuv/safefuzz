package fuzzingengine.operations;

import javax.json.JsonObject;

public abstract class JSONFuzzingOperation extends ValueFuzzingOperation {
	public String fuzzTransformString(String input) {
		return null;
		// TODO: deserialise it and reserialise
	}
	
	public abstract JsonObject fuzzTransformMessage(JsonObject msg);
}
