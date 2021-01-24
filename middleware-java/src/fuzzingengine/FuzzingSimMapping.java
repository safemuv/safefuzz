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
	
	public enum FuzzingNature {
		BINARY,
		CUSTOM_CONFIG,
		NO_MODIFICATIONS
	}
	
	public class VariableSpecification {
		private String component;
		private String variable;
		private String reflectionName;
		private VariableDirection dir;
		private Optional<String> binaryPath;
		private Optional<String> regexp;
		
		public VariableSpecification(String component, String variable, String reflectionName, VariableDirection dir, Optional<String> binaryPath, Optional<String> regexp) {
			this.component = component;
			this.variable = variable;
			this.reflectionName = reflectionName;
			this.dir = dir;
			this.binaryPath = binaryPath;
			this.regexp = regexp;
		}
		
		public Optional<String> getComponent() {
			return Optional.of(component);
		}
		
		public String getVariable() {
			return variable;
		}
		
		public Optional<String> getReflectionName() {
			return Optional.of(reflectionName);
		}
		
		public Optional<String> getRegexp() {
			return regexp;
		}
	}
	
	private class FuzzingComponentNatureInfo {
		private String name;
		private FuzzingNature nature;
		private Optional<String> classNameString;
		private Optional<String> fullPath;
		
		public FuzzingComponentNatureInfo(String name, FuzzingNature nature, Optional<String> classNameString, Optional<String> fullPath) {
			this.name = name;
			this.nature = nature;
			this.classNameString = classNameString;
			this.fullPath = fullPath;
		}
	}
	
	// Maps the component name to variable specification
	private HashMap<String, List<VariableSpecification>> records = new HashMap<String,List<VariableSpecification>>();
	// Maps the variable name to variable specifications
	private HashMap<String,VariableSpecification> recordsVariables = new HashMap<String,VariableSpecification>();
	private HashMap<String,FuzzingComponentNatureInfo> componentFuzzingInfo = new HashMap<String,FuzzingComponentNatureInfo>();
	
	public void addRecord(String component, String variable, String reflectionName, VariableDirection dir, Optional<String> binaryPath, Optional<String> regex) {
		if (!records.containsKey(component)) {
			records.put(component, new ArrayList<VariableSpecification>());
		}
		
		VariableSpecification vs = new VariableSpecification(component, variable, reflectionName, dir, binaryPath, regex);
		records.get(component).add(vs);
		recordsVariables.put(variable, vs);
	}
	
	public VariableSpecification getRecordForKey(String key) {
		return recordsVariables.get(key);
	}
	
	// Returns all the strings to mutate for a binary
	public HashMap<String,String> getBinaryChanges(String component, HashMap<String,String> inOutNames) {
		//	TODO: returning null hashmap for getBinaryChanges 
		HashMap<String,String> binaryVars = new HashMap<String,String>();
	
		for (Map.Entry<String,String> me : inOutNames.entrySet()) {
			String inkey = me.getKey();
			String outkey = me.getValue();
			if (isBinary(me.getKey())) {
				binaryVars.put(inkey, outkey);
			}
		}
		return binaryVars;
	}

	public boolean isBinary(String component) {
		// Get a record if it exists, indicate that the component nature is set to binary
		FuzzingComponentNatureInfo fcni = componentFuzzingInfo.get(component);
		if (fcni == null) {
			return false;
		} else {
			return (fcni.nature == FuzzingNature.BINARY);
		}
	}

	public String getFullPath(String component) throws MissingComponentPath {
		FuzzingComponentNatureInfo fcni = componentFuzzingInfo.get(component);
		if (fcni == null) {
			throw new MissingComponentPath(component);
		} else {
			Optional<String> p = fcni.fullPath;
			if (p.isPresent()) {
				return p.get();
			} else throw new MissingComponentPath(component);
		}
	}

	public void setComponentFuzzingInfo(String component, FuzzingNature nature, Optional<String> classNameForConfig, Optional<String> fullPath) {
		FuzzingComponentNatureInfo fcni = new FuzzingComponentNatureInfo(component, nature, classNameForConfig, fullPath);
		componentFuzzingInfo.put(component,fcni);
	}
}
