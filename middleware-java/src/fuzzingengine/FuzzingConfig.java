package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import fuzzingengine.FuzzingSimMapping.VariableDirection;
import fuzzingengine.operations.FuzzingOperation;

public class FuzzingConfig {
	private HashMap<String, FuzzingKeySelectionRecord> keyLookup = new LinkedHashMap<String, FuzzingKeySelectionRecord>();
	private HashMap<String, FuzzingKeySelectionRecord> keysByComponentLookup = new LinkedHashMap<String, FuzzingKeySelectionRecord>();
	private HashMap<String, FuzzingComponentSelectionRecord> componentLookup = new LinkedHashMap<String, FuzzingComponentSelectionRecord>();
	
	private HashMap<String, FuzzingMessageSelectionRecord> messageLookup = new LinkedHashMap<String, FuzzingMessageSelectionRecord>();
	private List<FuzzingKeySelectionRecord> keyRecords = new ArrayList<FuzzingKeySelectionRecord>();
	private List<FuzzingMessageSelectionRecord> messageRecords = new ArrayList<FuzzingMessageSelectionRecord>();
	
	public void addKeyRecord(FuzzingKeySelectionRecord fr) {
		keyRecords.add(fr);
		keyLookup.put(fr.getKey(), fr);
		System.out.println("adding key record: key " + fr.getKey());
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
	
	// This returns any component names that are selected by their key records selecting them for fuzzing
	public Set<String> getComponentsByKeyRecords() {
		System.out.println("keyRecords = " + keyRecords);
		return keyRecords.stream()
				.flatMap(c -> c.hasComponent() ? Stream.of(c.getComponent()) : Stream.empty()).collect(Collectors.toSet());
	}
	
	// This returns any component names that are selected by component records selecting them for fuzzing
	public Set<String> getComponents() {
		return componentLookup.keySet();
	}
	
	// This returns any components selected either directly, or selected because keys they contain are selected for fuzzing
	public Set<String> getComponentsByEither() {
		return Stream.concat(getComponents().stream(), getComponentsByKeyRecords().stream()).collect(Collectors.toSet());
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

	public Optional<FuzzingOperation> getOperationByKeyAndVehicle(String key, String vehicle, double time) {
		FuzzingKeySelectionRecord fr = keyLookup.get(key);
		if (fr != null) {
			if (fr.isReadyAtTime(time) && fr.hasVehicle(vehicle)) {
				return Optional.of(fr.getOperation());
			} else {
				//System.out.println("hasVehicle " + vehicle + " is false or timing not met");
			}
		}
		return Optional.empty();
	}

	public Optional<FuzzingOperation> getOperationByOutboundComponentAndVehicle(String componentName, String vehicle) {
		FuzzingComponentSelectionRecord fr = componentLookup.get(componentName);
		//System.out.println("componentLookup for " + componentName + " resulting in " + fr);
		if (fr != null) {
			if (fr.hasVehicle(vehicle)) {
				return Optional.of(fr.getOperation());
			}
		} 
		return Optional.empty();
	}
		
	public List<FuzzingKeySelectionRecord> getAllKeysByComponent(String component) {
		return keyRecords.stream().filter(kr -> kr.getComponent().equals(component)).collect(Collectors.toList());
	}
	
	// This returns any component names that are selected by their key records selecting them for fuzzing
	public Set<String> getComponentsByKeyRecordsForRobot(String rname) {
		System.out.println("keyRecords = " + keyRecords);
		return keyRecords.stream()
				.flatMap(r -> (r.hasComponent() && r.hasVehicle(rname)) ? Stream.of(r.getComponent()) : Stream.empty()).collect(Collectors.toSet());
	}
	
	// This returns any component names that are selected by component records selecting them for fuzzing on the given robot
	public Set<String> getComponentsForRobot(String rname) {
		return componentLookup.entrySet().stream()
				.filter(c -> c.getValue().hasVehicle(rname))
				.map(c -> c.getKey())
				.collect(Collectors.toSet());
	}
	
	// This returns any components selected either directly, or selected because keys they contain are selected for fuzzing
	public Set<String> getComponentsByEitherForRobot(String rname) {
		return Stream.concat(getComponentsForRobot(rname).stream(), 
							 getComponentsByKeyRecordsForRobot(rname).stream())
				.collect(Collectors.toSet());
	}
}