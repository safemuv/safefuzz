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
	private static final boolean DEBUG_JSON_OBJECT_RESTRUCTURING = false; 
	
	public static void printSpecFields(String [] specFields) {
		System.out.println("specfields length = " + specFields.length);
		for (int i = 0; i < specFields.length; i++) {
			System.out.println(i + "=" + specFields[i]);
		}
	}
	
	public static void insertPrimitiveType(JsonObjectBuilder builder, JsonValue source, String key, String fuzzed) {
		if (source.getValueType() == ValueType.STRING) {
			builder.add(key, fuzzed);
		}
		
		if (source.getValueType() == ValueType.NUMBER) {
			builder.add(key, Double.valueOf(fuzzed));
		}
		
		if (source.getValueType() == ValueType.ARRAY) {
			JsonReader jsonReader = Json.createReader(new StringReader(fuzzed));
			builder.add(key, jsonReader.readArray());
		}
	}
	
	public static JsonObject fuzzReplacement(JsonObject js, String [] specFields, ValueFuzzingOperation op) {
		//System.out.println("specFields = " + specFields);
		if (DEBUG_JSON_OBJECT_RESTRUCTURING) {
			printSpecFields(specFields);
		}
		
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
						JsonValue toFuzz = entry.getValue();
						String fuzzed = op.fuzzTransformString(toFuzz.toString());
			    		JsonReader jsonReader = Json.createReader(new StringReader(fuzzed));
			    		JsonObject jModified = jsonReader.readObject();
			    		builder.add(entry.getKey(), jModified);
					} else {
						builder.add(entry.getKey(), entry.getValue());
					}
				}
			} else {
				// Primitive object
				if (specFields.length == 1) {
					String lastSpec = specFields[0];
					if (lastSpec.equals(entry.getKey())) {
						// Found the element to fuzz
						JsonValue toFuzz = entry.getValue();
						String fuzzed = op.fuzzTransformString(toFuzz.toString());
						insertPrimitiveType(builder, toFuzz, entry.getKey(), fuzzed);		 
					}
				} else {
					builder.add(entry.getKey(), entry.getValue());
				}
			}
		}
		return builder.build();
	}
}
