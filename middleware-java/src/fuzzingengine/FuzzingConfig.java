package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import fuzzingengine.FuzzingSimMapping.VariableDirection;
import fuzzingengine.operations.FuzzingOperation;

class FuzzingConfig {
	private HashMap<String, FuzzingKeySelectionRecord> keyLookup = new LinkedHashMap<String, FuzzingKeySelectionRecord>();
	private HashMap<String, FuzzingKeySelectionRecord> keysByComponentLookup = new LinkedHashMap<String, FuzzingKeySelectionRecord>();
	private HashMap<String, FuzzingComponentSelectionRecord> componentLookup = new LinkedHashMap<String, FuzzingComponentSelectionRecord>();
	
	private HashMap<String, FuzzingMessageSelectionRecord> messageLookup = new LinkedHashMap<String, FuzzingMessageSelectionRecord>();
	private List<FuzzingKeySelectionRecord> keyRecords = new ArrayList<FuzzingKeySelectionRecord>();
	private List<FuzzingMessageSelectionRecord> messageRecords = new ArrayList<FuzzingMessageSelectionRecord>();
	
	public void addKeyRecord(FuzzingKeySelectionRecord fr) {
		keyRecords.add(fr);
		keyLookup.put(fr.getKey(), fr);
		if (fr.hasComponent()) {
			keysByComponentLookup.put(fr.getComponent(), fr);
		}
	}
	
	public void addMessageRecord(FuzzingMessageSelectionRecord mr) {
		messageRecords.add(mr);
		messageLookup.put(mr.getKey(), mr);
	}
	
	public Optional<Pattern> getPatternByKey(String key) {
		FuzzingKeySelectionRecord fr = keyLookup.get(key);
		if (fr != null) {
			return fr.getPattern();
		} else return Optional.empty();
	}
	
	public Optional<Map.Entry<Pattern,Integer>> getPatternAndGroupNum(String key) {
		FuzzingKeySelectionRecord fr = keyLookup.get(key);
		if (fr != null) {
			return fr.getPatternAndGroupNum();
		} else return Optional.empty();
	}
	
	public Optional<FuzzingOperation> getOperationByKey(String key) {
		FuzzingKeySelectionRecord fr = keyLookup.get(key);
		if (fr != null) {
			return Optional.of(fr.getOperation());
		} else return Optional.empty();
	}
	
	public Optional<FuzzingOperation> getOperationByComponent(String component) {
		FuzzingComponentSelectionRecord fr = componentLookup.get(component);
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
		return keyRecords.stream().map(c -> c.getComponent()).collect(Collectors.toSet());
	}
	
	public Optional<FuzzingOperation> hasMessageKey(String key) {
		FuzzingMessageSelectionRecord mr = messageLookup.get(key);
		if (mr != null) {
			return Optional.of(mr.getOperation());
		} else return Optional.empty();
	}
	
	public List<String> getMessageKeys(String robotName, VariableDirection dir) {
		List<String> out = new ArrayList<String>();
		
		for (FuzzingMessageSelectionRecord mr : messageRecords) {
			if (mr.getRobotFrom() == robotName && dir == VariableDirection.OUTBOUND) {
				out.add(mr.getKey());
			}
		
			if (mr.getRobotTo() == robotName && dir == VariableDirection.INBOUND) {
				out.add(mr.getKey());
			}
		}
		return out;
	}

	public void addComponentRecord(FuzzingComponentSelectionRecord cr) {
		componentLookup.put(cr.componentName, cr);
	}

	public Optional<FuzzingOperation> getOperationByKeyAndVehicle(String key, String vehicle) {
		// TODO: maybe lookup based on both key and vehicle
		FuzzingKeySelectionRecord fr = keyLookup.get(key);
		if (fr != null) {
			if (fr.hasVehicle(vehicle)) {
				return Optional.of(fr.getOperation());
			}
		} 
		return Optional.empty();
	}

	public Optional<FuzzingOperation> getOperationByComponentAndVehicle(String componentName, String vehicle) {
		FuzzingComponentSelectionRecord fr = componentLookup.get(componentName);
		if (fr != null) {
			if (fr.hasVehicle(vehicle)) {
				return Optional.of(fr.getOperation());
			}
		} 
		return Optional.empty();
	}
}