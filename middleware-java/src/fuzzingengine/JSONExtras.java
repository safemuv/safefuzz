package fuzzingengine;

import java.io.StringReader;
import java.util.Arrays;
import java.util.Map;

import javax.json.Json;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonReader;
import javax.json.JsonValue;
import javax.json.JsonValue.ValueType;

import fuzzingengine.operations.ValueFuzzingOperation;

public class JSONExtras {
	// TODO: use a JSonFuzzingOperation class
	public static JsonObject fuzzReplacement(JsonObject js, String [] specFields, ValueFuzzingOperation op) {
		JsonObjectBuilder builder = Json.createObjectBuilder();
		for (Map.Entry<String,JsonValue> entry : js.entrySet()){
			JsonValue jv = entry.getValue();
			
			if (jv.getValueType() == ValueType.OBJECT) {
				JsonObject jo = (JsonObject)jv;
				if (specFields.length > 1) {
					String[] newFields = Arrays.copyOfRange(specFields, 1, specFields.length);
					builder.add(entry.getKey(), fuzzReplacement(jo, newFields, op));
				} else {
					String lastSpec = specFields[0];
					if (lastSpec.equals(entry.getKey())) {
						// Found the element to fuzz
						String fuzzed = op.fuzzTransformString(js.toString());
			    		JsonReader jsonReader = Json.createReader(new StringReader(fuzzed));
			    		JsonObject jModified = jsonReader.readObject();
					} else {
						builder.add(entry.getKey(), entry.getValue());
					}
				}
			} else {
				// Primitive object - just copy it in
				builder.add(entry.getKey(), entry.getValue());
			}
		}
		return builder.build();
	}
}
