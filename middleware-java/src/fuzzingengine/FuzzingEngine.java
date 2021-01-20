package fuzzingengine;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

import java.lang.reflect.*;

import atlasdsl.Message;
import atlasdsl.Mission;
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
	
	public void addFuzzingKeyOperation(String fuzzingKey, int groupNum, FuzzingOperation op) {
		VariableSpecification vr = simmapping.getRecordForKey(fuzzingKey);
		FuzzingKeySelectionRecord fr = new FuzzingKeySelectionRecord(fuzzingKey, vr.getReflectionName(), vr.getComponent(), vr.getRegexp(),
				groupNum, op);
		confs.addKeyRecord(fr);
	}
	
	private void addFuzzingComponentOperation(String componentName, FuzzingOperation op) {
		// This operation has to produce the operation here
	}

	public void addFuzzingMessageOperation(String messageName, String messageFieldName, int groupNum, FuzzingOperation op) throws InvalidMessage {
		Message msg = m.getMessage(messageName);
		// TODO: handle case where the given message is not defined in the mission
		FuzzingMessageSelectionRecord mr = new FuzzingMessageSelectionRecord(messageFieldName, msg, op);
		// TODO: get the regex info from the simmapping for the message
		FuzzingKeySelectionRecord kr = new FuzzingKeySelectionRecord(fuzzingMessageKey, fuzzingMessageKeyReflection, regex, groupNum, op);
		confs.addKeyRecord(kr);
		confs.addMessageRecord(mr);
	}

	// TODO: this should be sensitive to the robot name and timing?
	public <E> Optional<FuzzingOperation> shouldFuzzCARSEvent(E event) {
		ATLASLog.logFuzzing("shouldFuzzCARSEvent called - " + event);
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cv = (CARSVariableUpdate) event;

			String key = cv.getKey();
			Optional<FuzzingOperation> op_o = confs.getOperationByKey(key);
			if (op_o.isPresent()) {
				return op_o;
			} else {
				Optional<String> componentName_o = cv.getSourceComponent();
				if (componentName_o.isPresent()) {
					String componentName = componentName_o.get();
					return confs.getOperationByComponent(componentName);
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

	public Set<String> getComponents() {
		// TODO: Return all selected components, either all components or those with a
		// selected key,
		// depending on mode
		return confs.getComponents();
	}

	public HashMap<String, String> getVariables() {
		return new HashMap<String, String>();
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

	public void setupFromFuzzingFile(String fileName) {
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
							addFuzzingKeyOperation(varName, groupNum, op);
							System.out.println("Installing fuzzing operation for " +varName + " - " + op);
						}
					}
					
					// Scan for component record in file
					if (fields[0].toUpperCase().equals("COMPONENT")) {
						// TODO: parse the vehicle names from here
						String componentName = fields[1];
						String vehicleNames = fields[2];
						String dirString = fields[3];
						String opClass = fields[4];
						String params = fields[5];
						Optional<FuzzingOperation> op_o = loadOperation(opClass, params);
						if (op_o.isPresent()) {
							FuzzingOperation op = op_o.get();
							addFuzzingComponentOperation(componentName, op);
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


}
