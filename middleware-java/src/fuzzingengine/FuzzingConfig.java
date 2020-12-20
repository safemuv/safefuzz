package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import fuzzingengine.operations.FuzzingOperation;

class FuzzingConfig {
	private HashMap<String, FuzzingKeySelectionRecord> keyLookup = new LinkedHashMap<String, FuzzingKeySelectionRecord>();
	private HashMap<String, FuzzingKeySelectionRecord> componentLookup = new LinkedHashMap<String, FuzzingKeySelectionRecord>();
	private HashMap<String, FuzzingKeySelectionRecord> Lookup = new LinkedHashMap<String, FuzzingKeySelectionRecord>();
	private List<FuzzingKeySelectionRecord> records = new ArrayList<FuzzingKeySelectionRecord>();

	// TODO: FuzzingComponentSelectionRecord
	public class FuzzingKeySelectionRecord {
		String key;
		Optional<String> reflectionKey;
		Optional<String> component;
		FuzzingOperation op;
		Optional<String> subfield;
		Optional<String> vehicleSelection;

		public FuzzingKeySelectionRecord(String key, Optional<String> reflectionKey, Optional<String> component, Optional<String> subfield,
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
	
	public void addRecord(FuzzingKeySelectionRecord fr) {
		records.add(fr);
		keyLookup.put(fr.getKey(), fr);
		if (fr.hasComponent()) {
			componentLookup.put(fr.getComponent(), fr);
		}
	}
	
	public Optional<FuzzingOperation> getOperationByKey(String key) {
		FuzzingKeySelectionRecord fr = keyLookup.get(key);
		if (fr != null) {
			return Optional.of(fr.getOperation());
		} else return Optional.empty();
	}
	
	public Optional<FuzzingOperation> getOperationByComponent(String component) {
		FuzzingKeySelectionRecord fr = componentLookup.get(component);
		if (fr != null) {
			return Optional.of(fr.getOperation());
		} else return Optional.empty();
	}
	
	public Optional<String> getReflectionKey(String key) {
		FuzzingKeySelectionRecord fr = keyLookup.get(key);
		if (fr != null) {
			return fr.getReflectionKey();
		} else return Optional.empty();
	}
	
	public Set<String> getComponents() {
		return records.stream().map(c -> c.getComponent()).collect(Collectors.toSet());
	}

}