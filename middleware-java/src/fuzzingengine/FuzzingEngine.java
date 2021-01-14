package fuzzingengine;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import atlasdsl.Message;
import fuzzingengine.FuzzingSimMapping.VariableDirection;
import fuzzingengine.operations.EventFuzzingOperation;
import fuzzingengine.operations.FuzzingOperation;
import fuzzingengine.operations.ValueFuzzingOperation;
import middleware.core.*;
import middleware.logging.ATLASLog;

public class FuzzingEngine {
	FuzzingConfig confs = new FuzzingConfig();
	FuzzingSimMapping simmapping = new FuzzingSimMapping();
	
	public void addFuzzingKeyOperation(String fuzzingKey, Optional<String> reflectionKey, Optional<String> component, Optional<String> regex, int groupNum, FuzzingOperation op) {
		FuzzingKeySelectionRecord fr = new FuzzingKeySelectionRecord(fuzzingKey, reflectionKey, component, regex, groupNum, op);
		confs.addKeyRecord(fr);
	}
	
	public void addFuzzingMessageOperation(String fuzzingMessageKey, Optional<String> fuzzingMessageKeyReflection, Message m, Optional<String> regex, int groupNum, FuzzingOperation op) {
		FuzzingMessageSelectionRecord mr = new FuzzingMessageSelectionRecord(fuzzingMessageKey, m, op);
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

	public static String replaceGroup(Pattern p, String source, int groupToReplace, ValueFuzzingOperation op) throws FuzzingEngineMatchFailure {
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
			EventFuzzingOperation eop = (EventFuzzingOperation)op;
			return eop.fuzzTransformEvent(event);
		} else {
			if (event instanceof CARSVariableUpdate) {
				CARSVariableUpdate cv = (CARSVariableUpdate) event;
				CARSVariableUpdate newUpdate = new CARSVariableUpdate(cv);
				ValueFuzzingOperation vop = (ValueFuzzingOperation)op;
				String key = cv.getKey();
				String v = cv.getValue();
				String newValue = v;
				// Need to also look up the group number
				Optional<Map.Entry<Pattern,Integer>> regexp_sel = confs.getPatternAndGroupNum(key);
				if (!regexp_sel.isPresent()) {
					newValue = vop.fuzzTransformString(v);
				} else {
					Pattern p = regexp_sel.get().getKey();
					Integer groupNum = regexp_sel.get().getValue();
					try {
						newValue = replaceGroup(p, v, groupNum,  vop);
					} catch (FuzzingEngineMatchFailure e) {
						ATLASLog.logFuzzing("FuzzingEngineMatchFailure - " + event + e);
					}
				}
				
				newUpdate.setValue(newValue); 
				return (E)newUpdate;
			} else {
				return event;
			}
		}
	}
	
	public FuzzingSimMapping getSimMapping() {
		return simmapping;
	}

	public Set<String> getComponents() {
		// TODO: Return all selected components, either all components or those with a selected key, 
		// depending on mode
		return confs.getComponents();
	}
	
	public HashMap<String,String> getVariables() {
		return new HashMap<String,String>();
	}

	public void setSimMapping(FuzzingSimMapping simMapping) {
		this.simmapping = simMapping;
	}

	public List<String> getMessageKeys(String robotName, VariableDirection dir) {
		//TODO: add these keys during generation process
		return confs.getMessageKeys(robotName, dir);
	}
}
