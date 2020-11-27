package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Optional;
import fuzzingengine.operations.FuzzingOperation;

class FuzzingConfig {
	
	private HashMap<String, FuzzingConfigRecord> keyLookup = new LinkedHashMap<String, FuzzingConfigRecord>();
	private HashMap<String, FuzzingConfigRecord> componentLookup = new LinkedHashMap<String, FuzzingConfigRecord>();
	private HashMap<String, FuzzingConfigRecord> Lookup = new LinkedHashMap<String, FuzzingConfigRecord>();
	private List<FuzzingConfigRecord> records = new ArrayList<FuzzingConfigRecord>();

	public class FuzzingConfigRecord {
		String key;
		Optional<String> reflectionKey;
		Optional<String> component;
		FuzzingOperation op;
		Optional<String> subfield;

		public FuzzingConfigRecord(String key, Optional<String> reflectionKey, Optional<String> component, Optional<String> subfield,
				FuzzingOperation op) {
			this.key = key;
			this.reflectionKey = reflectionKey;
			this.component = component;
			this.op = op;
			this.subfield = subfield;
		}

		public String getKey() {
			return key;
		}

		public Optional<String> getReflectionKey() {
			return reflectionKey;
		}

		public boolean hasComponent() {
			return component.isPresent();
		}
		
		public String getComponent() {
			return component.get();
		}

		public FuzzingOperation getOperation() {
			return op;
		}
	}
	
	public void addRecord(FuzzingConfigRecord fr) {
		records.add(fr);
		keyLookup.put(fr.getKey(), fr);
		if (fr.hasComponent()) {
			componentLookup.put(fr.getComponent(), fr);
		}
	}
	
	public Optional<FuzzingOperation> getOperationByKey(String key) {
		FuzzingConfigRecord fr = keyLookup.get(key);
		if (fr != null) {
			return Optional.of(fr.getOperation());
		} else return Optional.empty();
	}
	
	public Optional<FuzzingOperation> getOperationByComponent(String component) {
		FuzzingConfigRecord fr = componentLookup.get(component);
		if (fr != null) {
			return Optional.of(fr.getOperation());
		} else return Optional.empty();
	}
	
	public Optional<String> getReflectionKey(String key) {
		FuzzingConfigRecord fr = keyLookup.get(key);
		if (fr != null) {
			return fr.getReflectionKey();
		} else return Optional.empty();
	}

}