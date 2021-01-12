package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import fuzzingengine.FuzzingSimMapping.VariableDirection;
import fuzzingengine.operations.FuzzingOperation;

class FuzzingConfig {
	// TODO: merge all of these together?
	private HashMap<String, FuzzingKeySelectionRecord> keyLookup = new LinkedHashMap<String, FuzzingKeySelectionRecord>();
	private HashMap<String, FuzzingKeySelectionRecord> componentLookup = new LinkedHashMap<String, FuzzingKeySelectionRecord>();
	private HashMap<String, FuzzingMessageSelectionRecord> messageLookup = new LinkedHashMap<String, FuzzingMessageSelectionRecord>();
	private List<FuzzingKeySelectionRecord> keyRecords = new ArrayList<FuzzingKeySelectionRecord>();
	private List<FuzzingMessageSelectionRecord> messageRecords = new ArrayList<FuzzingMessageSelectionRecord>();
	
	// TODO: FuzzingComponentSelectionRecord?
	public void addKeyRecord(FuzzingKeySelectionRecord fr) {
		keyRecords.add(fr);
		keyLookup.put(fr.getKey(), fr);
		if (fr.hasComponent()) {
			componentLookup.put(fr.getComponent(), fr);
		}
	}
	
	public void addMessageRecord(FuzzingMessageSelectionRecord mr) {
		messageRecords.add(mr);
		messageLookup.put(mr.getKey(), mr);
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
}