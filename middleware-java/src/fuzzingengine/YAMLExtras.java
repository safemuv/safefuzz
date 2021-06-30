package fuzzingengine;

import java.util.Arrays;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.DoubleNode;
import com.fasterxml.jackson.databind.node.IntNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import fuzzingengine.operations.ValueFuzzingOperation;

public class YAMLExtras {
	private static final boolean DEBUG_YAML_OBJECT_RESTRUCTURING = true;
	
	public static void fuzzTransformYAML(ObjectMapper obj, JsonNode js, String [] specFields, ValueFuzzingOperation op) {
		System.out.println("js=" + js);
		if (specFields.length > 1) {
				String[] newFields = Arrays.copyOfRange(specFields, 1, specFields.length);
				String nextEntry = specFields[0];
				fuzzTransformYAML(obj, js.get(nextEntry), newFields, op);
		} else {
			String lastEntry = specFields[0];
			ObjectNode jsObj = (ObjectNode)js;
			JsonNode toFuzz = jsObj.get(lastEntry);	
			String fuzzed = op.fuzzTransformString(toFuzz.toString());
			System.out.println("fuzzed="+fuzzed);
			try {
				if (toFuzz.isObject()) {
					jsObj.set(lastEntry, (ObjectNode)obj.readTree(fuzzed));
				} 
				
				if (toFuzz.isArray()) {
					jsObj.set(lastEntry, (ArrayNode)obj.readTree(fuzzed));
				}
				
				if (toFuzz.isInt()) {
					jsObj.set(lastEntry, (IntNode)obj.readTree(fuzzed));
				}
				
				if (toFuzz.isDouble()) {
					jsObj.set(lastEntry, (DoubleNode)obj.readTree(fuzzed));
				}
			} catch (JsonProcessingException e) {
				e.printStackTrace();
			}
		}
	}
}
