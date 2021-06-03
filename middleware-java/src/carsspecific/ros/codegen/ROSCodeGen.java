package carsspecific.ros.codegen;

import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.operations.FuzzingOperation;
import fuzzingengine.operations.ValueFuzzingOperation;
import middleware.atlascarsgenerator.*;
import middleware.atlascarsgenerator.carsmapping.*;

import java.io.File;
import java.io.IOException;
import java.io.StringReader;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import javax.json.Json;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonReader;
import javax.json.JsonValue;
import javax.json.JsonValue.ValueType;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.fasterxml.jackson.dataformat.yaml.YAMLGenerator.Feature;

import atlasdsl.*;
import carsspecific.ros.rosmapping.ROSSimulation;

public class ROSCodeGen extends CARSCodeGen {
	private static final boolean DEBUG_YAML_OBJECT_RESTRUCTURING = true;

	// TODO: how to specify the sensor behaviour
	public ROSCodeGen(Mission m, Optional<FuzzingEngine> fe_o) {
		super(m, fe_o);
	}
	
	public CARSSimulation convertDSL() throws ConversionFailed {
		CARSSimulation rossim = new ROSSimulation();
		transformAllFiles();
		return rossim;	
	}
	
	public static JsonObject fuzzTransformYAML(JsonObject js, String [] specFields, ValueFuzzingOperation op) {
		//System.out.println("specFields = " + specFields);
		//if (DEBUG_YAML_OBJECT_RESTRUCTURING) {
//			printSpecFields(specFields);
		//}
		
		JsonObjectBuilder builder = Json.createObjectBuilder();
		for (Map.Entry<String,JsonValue> entry : js.entrySet()){
			JsonValue jv = entry.getValue();
			
			if (jv.getValueType() == ValueType.OBJECT) {
				JsonObject jo = (JsonObject)jv;
				if (specFields.length > 1) {
					String[] newFields = Arrays.copyOfRange(specFields, 1, specFields.length);
					builder.add(entry.getKey(), fuzzTransformYAML(jo, newFields, op));
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
				// Primitive object - just copy it in
				builder.add(entry.getKey(), entry.getValue());
			}
		}
		return builder.build();
	}
	
	public static void transformFiles(Set<FuzzingKeySelectionRecord> recs) {
		ObjectMapper mapper = new ObjectMapper(new YAMLFactory().disable(Feature.WRITE_DOC_START_MARKER));
		mapper.findAndRegisterModules();
		try {
			// Iterate over all the fuzzing key selection records
			for (FuzzingKeySelectionRecord r : recs) {
				// TODO: only those with keys on environmental components should be considered
				FuzzingOperation op = r.getOperation();
				String filename = r.getKey();
				Object internalSpec = r.getGroupNum();
				File f = new File(filename);
				Object res = mapper.readTree(new File(filename));
				String [] specFields = ((String)internalSpec).split("\\.");
				System.out.println("Res class=" + res.getClass());
				
				// Only ValueFuzzingOperations can be applied here... it only makes sense for them
				// to be used, since others cannot be sensibly applied 
				if (op instanceof ValueFuzzingOperation) {
					//fuzzTransformYAML(res, specFields);
				}
				
				// TODO: transform according to the given fuzzing operations
				mapper.writeValue(f, res);
			}
			

			
		} catch (JsonProcessingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void transformAllFiles() {
		if (fe_o.isPresent()) {
			FuzzingEngine fe = fe_o.get();
			Set<FuzzingKeySelectionRecord> records = fe.getAllEnvironmentalKeys();
			transformFiles(records);
		}
		
	}
}
