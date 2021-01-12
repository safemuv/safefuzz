package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;

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
	
	public void addFuzzingKeyOperation(String fuzzingKey, Optional<String> reflectionKey, Optional<String> component, FuzzingOperation op) {
		FuzzingKeySelectionRecord fr = new FuzzingKeySelectionRecord(fuzzingKey, reflectionKey, component, Optional.empty(), op);
		confs.addKeyRecord(fr);
	}
	
	public void addFuzzingMessageOperation(String fuzzingMessageKey, Optional<String> fuzzingMessageKeyReflection, Message m, FuzzingOperation op) {
		FuzzingMessageSelectionRecord mr = new FuzzingMessageSelectionRecord(fuzzingMessageKey, m, op);
		FuzzingKeySelectionRecord kr = new FuzzingKeySelectionRecord(fuzzingMessageKey, fuzzingMessageKeyReflection, op);
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

	public <E> E fuzzTransformEvent(E event, FuzzingOperation op) {
		if (op.isEventBased()) {
			EventFuzzingOperation eop = (EventFuzzingOperation)op;
			return eop.fuzzTransformEvent(event);
		} else {
			if (event instanceof CARSVariableUpdate) {
				CARSVariableUpdate cv = (CARSVariableUpdate) event;
				CARSVariableUpdate newUpdate = new CARSVariableUpdate(cv);
				ValueFuzzingOperation vop = (ValueFuzzingOperation)op;
				String v = cv.getValue();
				String newValue = vop.fuzzTransformString(v);
				// TODO: handle extracting chunks of string here - regex/grammar based
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
