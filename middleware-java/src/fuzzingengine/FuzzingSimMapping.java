package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

// Stores all the information for a simulator mapping
public class FuzzingSimMapping {

	public enum VariableDirection {
		INBOUND,
		OUTBOUND
	}
	
	public class VariableSpecification {
		private String component;
		private String variable;
		private String reflectionName;
		// TODO: encode the type definition
		private VariableDirection dir;
		private Optional<String> binaryPath;
		
		public VariableSpecification(String component, String variable, String reflectionName, VariableDirection dir, Optional<String> binaryPath) {
			this.component = component;
			this.variable = variable;
			this.reflectionName = reflectionName;
			this.dir = dir;
			this.binaryPath = binaryPath;
		}
	}
	
	// Maps the component name to variable specification
	private HashMap<String, List<VariableSpecification>> records = new HashMap<String,List<VariableSpecification>>();
	
	public void addRecord(String component, String variable, String reflectionName, VariableDirection dir, Optional<String> binaryPath) {
		if (!records.containsKey(component)) {
			records.put(component, new ArrayList<VariableSpecification>());
		}
		
		VariableSpecification vs = new VariableSpecification(component, variable, reflectionName, dir, binaryPath);
		records.get(component).add(vs);
	}
	
	
	// Returns all the strings to mutate for a binary
	public HashMap<String,String> getBinaryChanges(String component, HashMap<String,String> inNames) {
		//	TODO: returning null hashmap for getBinaryChanges 
		// replace this with filtering the input names to determine any defined as binary
		System.out.println("TODO: Define the binary changes from FuzzingSimMapping");
		HashMap<String,String> binaryVars = new HashMap<String,String>();
	
		for (Map.Entry<String,String> me : inNames.entrySet()) {
			String inkey = me.getKey();
			String outkey = me.getValue();
			if (isBinary(me.getKey())) {
				binaryVars.put(inkey, outkey);
			}
		}
		return binaryVars;
	}

	private boolean isBinary(String key) {
		return true;
	}

	public String getFullPath(String component) {
		// TODO Auto-generated method stub
		System.out.println("TODO: Define getFullPath");
		return "/tmp/uSimMarine";
	}
}
