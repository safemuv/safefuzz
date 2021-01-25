package fuzzingengine;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;


import java.lang.reflect.*;

import atlasdsl.Message;
import atlasdsl.Mission;
import atlasdsl.Robot;
import fuzzingengine.FuzzingSimMapping.VariableDirection;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.operations.EventFuzzingOperation;
import fuzzingengine.operations.FuzzingOperation;
import fuzzingengine.operations.ValueFuzzingOperation;
import middleware.core.*;
import middleware.logging.ATLASLog;

public class FuzzingEngine {
	Mission m;
	FuzzingConfig confs = new FuzzingConfig();
	FuzzingSimMapping simmapping = new FuzzingSimMapping();

	public FuzzingEngine(Mission m) {
		this.m = m;
	}
	
	private List<Robot> getVehicles(String vehicleList) throws MissingRobot {
		if (vehicleList.toUpperCase().equals("ALL")) {
			return m.getAllRobots();
		} else if (vehicleList.equals("")) {
			return m.getAllRobots();
		} else {
			List<String> vehicleNames = Arrays.asList(vehicleList.split("\\|"));
			// Check for the missing robots in the model
			List<Robot> robots = new ArrayList<Robot>();
			for (String vn : vehicleNames) {
				Robot r = m.getRobot(vn);
				if (r != null) {
					robots.add(r);
				} else {
					throw new MissingRobot(vn);
				}
			}
			return robots;
		}
	}
	
	public void addFuzzingKeyOperation(String fuzzingKey, String vehicleNameList, int groupNum, FuzzingOperation op) throws MissingRobot {
		VariableSpecification vr = simmapping.getRecordForKey(fuzzingKey);
		List<Robot> vehicles = getVehicles(vehicleNameList);
		FuzzingKeySelectionRecord fr = new FuzzingKeySelectionRecord(fuzzingKey, vr.getReflectionName(), vr.getComponent(), vr.getRegexp(),
				groupNum, op, vehicles);
		confs.addKeyRecord(fr);
	}
	
	private void addFuzzingComponentOperation(String componentName, FuzzingOperation op, String vehicleNameList) throws MissingRobot {
		List<Robot> vehicles = getVehicles(vehicleNameList);
		FuzzingComponentSelectionRecord cr = new FuzzingComponentSelectionRecord(componentName, op, vehicles);
		confs.addComponentRecord(cr);
	}

	public void addFuzzingMessageOperation(String messageName, String messageFieldName, int groupNum, FuzzingOperation op) throws InvalidMessage {
		Message msg = m.getMessage(messageName);
		if (msg == null) {
			throw new InvalidMessage(messageName, "Message not in model");
		} else {
			VariableSpecification vr = simmapping.getRecordForKey(messageFieldName);
			if (vr == null) {
				throw new InvalidMessage(messageName, "Simmapping key not present for message field name " + messageFieldName);
			} else {
				// TODO: handle case where the given message is not defined in the mission
				FuzzingMessageSelectionRecord mr = new FuzzingMessageSelectionRecord(messageFieldName, msg, op);
				FuzzingKeySelectionRecord kr = new FuzzingKeySelectionRecord(messageFieldName, vr.getReflectionName(), vr.getRegexp(), groupNum, op);
				confs.addKeyRecord(kr);
				confs.addMessageRecord(mr);
			}
		}
	}

	// TODO: this should be sensitive to the robot name and timing?
	public <E> Optional<FuzzingOperation> shouldFuzzCARSEvent(E event) {
		ATLASLog.logFuzzing("shouldFuzzCARSEvent called - " + event);
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cv = (CARSVariableUpdate) event;
			String vehicle = cv.getVehicleName();
			String key = cv.getKey();
			Optional<FuzzingOperation> op_o = confs.getOperationByKeyAndVehicle(key,vehicle);
			if (op_o.isPresent()) {
				return op_o;
			} else {
				Optional<String> componentName_o = cv.getSourceComponent();
				if (componentName_o.isPresent()) {
					String componentName = componentName_o.get();
					return confs.getOperationByComponentAndVehicle(componentName,vehicle);
				} else {
					return confs.hasMessageKey(key);
				}
			}
		} else {
			return Optional.empty();
		}
	}

	public <E> Optional<String> shouldReflectBackToCARS(E event) {
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cv = (CARSVariableUpdate) event;
			String k = cv.getKey();
			return confs.getReflectionKey(k);
		}
		return Optional.empty();
	}

	public static String replaceGroup(Pattern p, String source, int groupToReplace, ValueFuzzingOperation op)
			throws FuzzingEngineMatchFailure {
		Matcher m = p.matcher(source);
		if (!m.find()) {
			throw new FuzzingEngineMatchFailure(p, source);
		} else {
			String v = m.group(groupToReplace);
			String newV = op.fuzzTransformString(v);
			return new StringBuilder(source).replace(m.start(groupToReplace), m.end(groupToReplace), newV).toString();
		}
	}

	public <E> E fuzzTransformEvent(E event, FuzzingOperation op) {
		if (op.isEventBased()) {
			EventFuzzingOperation eop = (EventFuzzingOperation) op;
			return eop.fuzzTransformEvent(event);
		} else {
			if (event instanceof CARSVariableUpdate) {
				CARSVariableUpdate cv = (CARSVariableUpdate) event;
				CARSVariableUpdate newUpdate = new CARSVariableUpdate(cv);
				ValueFuzzingOperation vop = (ValueFuzzingOperation) op;
				String key = cv.getKey();
				String v = cv.getValue();
				String newValue = v;
				// Need to also look up the group number
				Optional<Map.Entry<Pattern, Integer>> regexp_sel = confs.getPatternAndGroupNum(key);
				if (!regexp_sel.isPresent()) {
					newValue = vop.fuzzTransformString(v);
				} else {
					Pattern p = regexp_sel.get().getKey();
					Integer groupNum = regexp_sel.get().getValue();
					try {
						newValue = replaceGroup(p, v, groupNum, vop);
					} catch (FuzzingEngineMatchFailure e) {
						ATLASLog.logFuzzing("FuzzingEngineMatchFailure - " + event + e);
					}
				}

				newUpdate.setValue(newValue);
				return (E) newUpdate;
			} else {
				return event;
			}
		}
	}

	public FuzzingSimMapping getSimMapping() {
		return simmapping;
	}
	
	public FuzzingConfig getConfig() {
		return confs;
	}

	public HashMap<String, String> getVariables() {
		// Need to return: any variables from components enabled for binary fuzzing
		// With either the component enabled for fuzzing
		// Or the individual variable enabled
		HashMap<String, String> vars = new HashMap<String,String>();
		//for (confs.getAllKeysByComponent(component)) {
		//	confs.getAllKeysByComponent
		//}
		
		return vars;
		
	}

	public void setSimMapping(FuzzingSimMapping simMapping) {
		this.simmapping = simMapping;
	}

	public List<String> getMessageKeys(String robotName, VariableDirection dir) {
		// TODO: add these keys during generation process
		return confs.getMessageKeys(robotName, dir);
	}
	
	public Optional<FuzzingOperation> loadOperation(String className, String params) {
		try {
			Class<?> c = Class.forName("fuzzingengine.operations." + className);
			System.out.println("className for operation " + className);
			Method method = c.getDeclaredMethod("createFromParamString", String.class);
			Object res = method.invoke(null, params);
			if (res instanceof FuzzingOperation) {
				return Optional.of((FuzzingOperation)res);
			} else {
				return Optional.empty();
			}
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		} catch (NoSuchMethodException e) {
			e.printStackTrace();
		} catch (SecurityException e) {
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
		} catch (InvocationTargetException e) {
			e.printStackTrace();
		}
		return Optional.empty();
	}

	public void setupFromFuzzingFile(String fileName, Mission m) {
		System.out.println("setupFromFuzzingFile - " + fileName);
		try {
			Files.readAllLines(Paths.get(fileName)).forEach(line -> {
				if (line.charAt(0) != '#') {
					String fields [] = line.split(",");
					System.out.println("0 - " + fields[0]);
					int i = 0;
					
					for (final String token : fields) {
						if (!token.isEmpty()) {
							System.out.println("Token:" + i + " " + token);
						}
						i++;
					}
					
					// Scan for key record in file
					if (fields[0].toUpperCase().equals("KEY")) {
						System.out.println("KEY based fuzzing");
						String  varName = fields[1];
						// TODO: parse the vehicle names from here
						String vehicleNames = fields[2];
						Integer groupNum = Integer.valueOf(fields[3]);
						String opClass = fields[4];
						String params = fields[5];
						
						Optional<FuzzingOperation> op_o = loadOperation(opClass, params);
						if (op_o.isPresent()) {
							FuzzingOperation op = op_o.get();
							try {
								addFuzzingKeyOperation(varName, vehicleNames, groupNum, op);
							} catch (MissingRobot e) {
								System.out.println("Missing robot: name = " + e.getName());
								e.printStackTrace();
							}
							System.out.println("Installing fuzzing operation for " +varName + " - " + op);
						}
					}
					
					// Scan for component record in file
					if (fields[0].toUpperCase().equals("COMPONENT")) {
						// TODO: parse the vehicle names from here
						String componentName = fields[1];
						String vehicleNames = fields[2];
						String dirString = fields[3];
						// TODO: not using the direction string yet
						String opClass = fields[4];
						String params = fields[5];
						Optional<FuzzingOperation> op_o = loadOperation(opClass, params);
						if (op_o.isPresent()) {
							FuzzingOperation op = op_o.get();
							try {
								addFuzzingComponentOperation(componentName, op, vehicleNames);
							} catch (MissingRobot e) {
								e.printStackTrace();
							}
							System.out.println("Installing fuzzing operation for " + componentName + " - " + op);
						}
					}
					
					// Scan for message
					if (fields[0].toUpperCase().equals("MESSAGE")) {
						System.out.println("Implement message-based fuzzing reader");
						String messageName = fields[1];
						// No vehicle names for messages - since they are pre-defined?
						String messageFieldName = fields[2];
						Integer groupNum = Integer.valueOf(fields[3]);
						String opClass = fields[4];
						String params = fields[5];
						Optional<FuzzingOperation> op_o = loadOperation(opClass, params);
						if (op_o.isPresent()) {
							FuzzingOperation op = op_o.get();
							// Where to get the regexp for the message fields? - in the model
							try {
								addFuzzingMessageOperation(messageName, messageFieldName, groupNum, op);
							} catch (InvalidMessage e) {
								System.out.println("Invalid message name: " + e.getMessageName() + " - " + e.getReason());
								e.printStackTrace();
								// TODO: raise exception to indicate the conversion failed
							}
							System.out.println("Installing fuzzing operation for " + messageName + ":" + messageFieldName + " - " + op);
						}
					}
					

				}
			});
		} catch (IOException ex) {
			ex.printStackTrace();
		}
	}
	
	public Map<String,String> getAllChanges(String component) {
		Map<String,String> inOut = new HashMap<String,String>();
		List<FuzzingKeySelectionRecord> recs = getConfig().getAllKeysByComponent(component);
		for (FuzzingKeySelectionRecord kr : recs) {
			if (kr.getReflectionKey().isPresent()) {
				inOut.put(kr.getKey(), kr.getReflectionKey().get());
			}
		}
		return inOut;
	}
}
