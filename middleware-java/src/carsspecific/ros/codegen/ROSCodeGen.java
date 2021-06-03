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

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.fasterxml.jackson.dataformat.yaml.YAMLGenerator.Feature;

import atlasdsl.*;
import carsspecific.ros.rosmapping.ROSSimulation;

public class ROSCodeGen extends CARSCodeGen {
	private static final boolean DEBUG_YAML_OBJECT_RESTRUCTURING = true;
	private static ObjectMapper obj = new ObjectMapper();

	public ROSCodeGen(Mission m, Optional<FuzzingEngine> fe_o) {
		super(m, fe_o);
		
	}
	
	public CARSSimulation convertDSL() throws ConversionFailed {
		CARSSimulation rossim = new ROSSimulation();
		transformAllFiles();
		return rossim;	
	}
	
	// TODO: factor out to YAMLHelper or YAMLExtras
	public static void fuzzTransformYAML(JsonNode js, String [] specFields, ValueFuzzingOperation op) {
		if (specFields.length > 1) {
				String[] newFields = Arrays.copyOfRange(specFields, 1, specFields.length);
				String nextEntry = specFields[0];
				fuzzTransformYAML(js.get(nextEntry), newFields, op);
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
			} catch (JsonProcessingException e) {
				e.printStackTrace();
			}
		}
	}
	
	public static void transformFiles(FuzzingEngine fe, Set<FuzzingKeySelectionRecord> recs) {
		ObjectMapper mapper = new ObjectMapper(new YAMLFactory().disable(Feature.WRITE_DOC_START_MARKER));
		mapper.findAndRegisterModules();
		try {
			// Iterate over all the fuzzing key selection records
			for (FuzzingKeySelectionRecord r : recs) {
				// TODO: only those with keys on environmental components should be considered
				FuzzingOperation op = r.getOperation();
				String filename = fe.getFilenameForKey(r.getKey());
				
				Object internalSpec = r.getGroupNum();
				File f = new File(filename);
				JsonNode res = (JsonNode)mapper.readTree(new File(filename));
				String [] specFields = ((String)internalSpec).split("\\.");
				//System.out.println("Res class=" + res.getClass());
				
				// Only ValueFuzzingOperations can be applied here... it only makes sense for them
				// to be used, since others such as delay cannot be sensibly applied 
				if (op instanceof ValueFuzzingOperation) {
					ValueFuzzingOperation opV = (ValueFuzzingOperation)op;
					fuzzTransformYAML(res, specFields, opV);
				}
				// Write back the modified file
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
			System.out.println("transformAllFiles = " + records);
			transformFiles(fe, records);
		}
		
	}
}
