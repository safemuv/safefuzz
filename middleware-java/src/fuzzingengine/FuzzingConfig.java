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
import middleware.gui.GUITest;

public class FuzzingConfig {
	private HashMap<String, List<FuzzingKeySelectionRecord>> keyLookup = new LinkedHashMap<String, List<FuzzingKeySelectionRecord>>();
	private HashMap<String, List<FuzzingKeySelectionRecord>> keysByComponentLookup = new LinkedHashMap<String, List<FuzzingKeySelectionRecord>>();
	private HashMap<String, FuzzingComponentSelectionRecord> componentLookup = new LinkedHashMap<String, FuzzingComponentSelectionRecord>();

	private HashMap<String, FuzzingMessageSelectionRecord> messageLookup = new LinkedHashMap<String, FuzzingMessageSelectionRecord>();
	private List<FuzzingKeySelectionRecord> keyRecords = new ArrayList<FuzzingKeySelectionRecord>();
	private List<FuzzingMessageSelectionRecord> messageRecords = new ArrayList<FuzzingMessageSelectionRecord>();

	public void addKeyRecord(FuzzingKeySelectionRecord fr) {
		String key = fr.getKey();
		keyRecords.add(fr);

		if (keyLookup.get(key) == null) {
			keyLookup.put(key, new ArrayList<FuzzingKeySelectionRecord>());
		}

		keyLookup.get(key).add(fr);
		System.out.println("adding key record: key " + fr.getKey());

		if (fr.hasComponent()) {
			String cKey = fr.getComponent();
			if (keysByComponentLookup.get(cKey) == null) {
				keysByComponentLookup.put(cKey, new ArrayList<FuzzingKeySelectionRecord>());
			}

			keysByComponentLookup.get(cKey).add(fr);
		}
	}

	public void addMessageRecord(FuzzingMessageSelectionRecord mr) {
		messageRecords.add(mr);
		messageLookup.put(mr.getKey(), mr);
	}

	public List<FuzzingKeySelectionRecord> getRecordsByKey(String key) {
		return keyLookup.get(key);
	}

//	public Optional<Pattern> getPatternByKey(String key) {
//		FuzzingKeySelectionRecord fr = keyLookup.get(key);
//		if (fr != null) {
//			return fr.getPattern();
//		} else return Optional.empty();
//	}
//	
//	public Optional<Object> getJSONStructure(String key) {
//		FuzzingKeySelectionRecord fr = keyLookup.get(key);
//		if (fr != null) {
//			Object o = fr.getGroupNum();
//			return Optional.of(o);
//		} else {
//			return Optional.empty();
//		}
//	}
//	
//	public Optional<Map.Entry<Pattern,Object>> getPatternAndGroupStructure(String key) {
//		FuzzingKeySelectionRecord fr = keyLookup.get(key);
//		if (fr != null) {
//			return fr.getPatternAndGroupStructure();
//		} else return Optional.empty();
//	}
//	
//	public Optional<FuzzingOperation> getOperationByKey(String key) {
//		FuzzingKeySelectionRecord fr = keyLookup.get(key);
//		if (fr != null) {
//			return Optional.of(fr.getOperation());
//		} else return Optional.empty();
//	}

//	public Optional<FuzzingOperation> getOperationByComponent(String component) {
//		FuzzingComponentSelectionRecord fr = componentLookup.get(component);
//		if (fr != null) {
//			return Optional.of(fr.getOperation());
//		} else return Optional.empty();
//	}

//	public Optional<String> getReflectionKey(String key) {
//		FuzzingKeySelectionRecord fr = keyLookup.get(key);
//		if (fr != null) {
//			return fr.getReflectionKey();
//		} else return Optional.empty();
//	}

	// This returns any component names that are selected by their key records
	// selecting them for fuzzing
	public Set<String> getComponentsByKeyRecords() {
		System.out.println("keyRecords = " + keyRecords);
		return keyRecords.stream().flatMap(c -> c.hasComponent() ? Stream.of(c.getComponent()) : Stream.empty())
				.collect(Collectors.toSet());
	}

	// This returns any component names that are selected by component records
	// selecting them for fuzzing
	public Set<String> getComponents() {
		return componentLookup.keySet();
	}

	// This returns any components selected either directly, or selected because
	// keys they contain are selected for fuzzing
	public Set<String> getComponentsByEither() {
		return Stream.concat(getComponents().stream(), getComponentsByKeyRecords().stream())
				.collect(Collectors.toSet());
	}

	public Optional<FuzzingOperation> hasMessageKey(String key) {
		FuzzingMessageSelectionRecord mr = messageLookup.get(key);
		if (mr != null) {
			return Optional.of(mr.getOperation());
		} else
			return Optional.empty();
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

//	public void addComponentRecord(FuzzingComponentSelectionRecord cr) {
//		componentLookup.put(cr.componentName, cr);
//	}

	// TODO: This method should be pulled into the fuzzing engine itself
	public List<FuzzingOperation> getOperationsByKeyAndVehicle(String key, String vehicle, double time) {
		List<FuzzingKeySelectionRecord> frs = keyLookup.get(key);
		GUITest g = GUITest.getGUI();
		List<FuzzingOperation> res = new ArrayList<FuzzingOperation>();

		for (FuzzingKeySelectionRecord fr : frs) {
			System.out.println("keyLookup=" + keyLookup + ",key=" + key + ",fr=" + fr);
			if (fr.isReadyAtTime(time, vehicle) && fr.hasVehicle(vehicle)) {
				Optional<FuzzingOperation> opRes = Optional.of(fr.getOperation());
				// Update the GUI here - to indicate active for this key
				if (g != null) {
					g.addFuzzingKeyState(key, vehicle, opRes.toString());
				}

				if (opRes.isPresent()) {
					res.add(fr.getOperation());
				}

			} else {
				// System.out.println("hasVehicle " + vehicle + " is false or timing not met");
			}
		}

		if (g != null) {
			g.setFuzzingKeyState(key, vehicle, "---");
		}
		return res;
	}

	// TODO: This method should be pulled into the fuzzing engine itself
	public List<FuzzingKeySelectionRecord> getRecordsByKeyAndVehicle(String key, String vehicle, double time) {
		List<FuzzingKeySelectionRecord> frs = keyLookup.get(key);
		GUITest g = GUITest.getGUI();
		List<FuzzingKeySelectionRecord> res = new ArrayList<FuzzingKeySelectionRecord>();

		if (frs != null) {
			for (FuzzingKeySelectionRecord fr : frs) {
				System.out.println("keyLookup=" + keyLookup + ",key=" + key + ",fr=" + fr);
				if (fr.isReadyAtTime(time, vehicle) && fr.hasVehicle(vehicle)) {
					Optional<FuzzingOperation> opRes = Optional.of(fr.getOperation());
					// Update the GUI here - to indicate active for this key
					if (g != null) {
						g.addFuzzingKeyState(key, vehicle, opRes.toString());
					}

					if (opRes.isPresent()) {
						res.add(fr);
					}

				} else {
					// System.out.println("hasVehicle " + vehicle + " is false or timing not met");
				}
			}
		}

		if (g != null) {
			g.setFuzzingKeyState(key, vehicle, "---");
		}
		return res;
	}

//	public Optional<FuzzingOperation> getOperationByOutboundComponentAndVehicle(String componentName, String vehicle) {
//		FuzzingComponentSelectionRecord fr = componentLookup.get(componentName);
//		//System.out.println("componentLookup for " + componentName + " resulting in " + fr);
//		if (fr != null) {
//			if (fr.hasVehicle(vehicle)) {
//				return Optional.of(fr.getOperation());
//			}
//		} 
//		return Optional.empty();
//	}

	public List<FuzzingKeySelectionRecord> getAllKeysByComponent(String component) {
		return keyRecords.stream().filter(kr -> kr.getComponent().equals(component)).collect(Collectors.toList());
	}

	// This returns any component names that are selected by their key records
	// selecting them for fuzzing
	public Set<String> getComponentsByKeyRecordsForRobot(String rname) {
		System.out.println("keyRecords = " + keyRecords);
		return keyRecords.stream()
				.flatMap(r -> (r.hasComponent() && r.hasVehicle(rname)) ? Stream.of(r.getComponent()) : Stream.empty())
				.collect(Collectors.toSet());
	}

//	// This returns any component names that are selected by component records selecting them for fuzzing on the given robot
	public Set<String> getComponentsForRobot(String rname) {
		return componentLookup.entrySet().stream().filter(c -> c.getValue().hasVehicle(rname)).map(c -> c.getKey())
				.collect(Collectors.toSet());
	}

//	// This returns any components selected either directly, or selected because keys they contain are selected for fuzzing
	public Set<String> getComponentsByEitherForRobot(String rname) {
		return Stream.concat(getComponentsForRobot(rname).stream(), getComponentsByKeyRecordsForRobot(rname).stream())
				.collect(Collectors.toSet());
	}

	public Set<String> getAllKeys() {
		return keyLookup.entrySet().stream().map(c -> c.getKey()).collect(Collectors.toSet());
	}

	public List<FuzzingKeySelectionRecord> getAllKeyRecords() {
		return keyRecords;
	}

	public HashMap<String, List<FuzzingKeySelectionRecord>> getKeyLookup() {
		return keyLookup;
	}
}
