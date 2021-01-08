package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import atlasdsl.Message;
import fuzzingengine.FuzzingSimMapping.VariableDirection;
import fuzzingengine.operations.FuzzingOperation;
import middleware.core.*;
import middleware.logging.ATLASLog;

public class FuzzingEngine {
	FuzzingConfig confs = new FuzzingConfig();
	FuzzingSimMapping simmapping = new FuzzingSimMapping();
	
	private FuzzingSelectionType fuzzSelType;

	public FuzzingEngine() {
		fuzzSelType = FuzzingSelectionType.LOCAL_BY_SPECIFIC_KEYS;
	}

	public FuzzingEngine(String filename) {
		fuzzSelType = FuzzingSelectionType.LOCAL_BY_SPECIFIC_KEYS;
	}
	
	public void setFuzzSelectionType(FuzzingSelectionType fuzzSelType) {
		this.fuzzSelType = fuzzSelType;
	}
	
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
	
	// TODO: how to handle subfields?
	public <E> Optional<FuzzingOperation> shouldFuzzCARSEvent(E event) {
		ATLASLog.logFuzzing("shouldFuzzCARSEvent called - " + event);
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cv = (CARSVariableUpdate) event;

			if (fuzzSelType == FuzzingSelectionType.LOCAL_BY_SPECIFIC_KEYS) {
				String k = cv.getKey();
				return confs.getOperationByKey(k);
			}
			
			if (fuzzSelType == FuzzingSelectionType.LOCAL_BY_SPECIFIC_SOURCE) {
				Optional<String> componentName_o = cv.getSourceComponent();
				if (componentName_o.isPresent()) {
					String componentName = componentName_o.get();
					return confs.getOperationByComponent(componentName);
				} else {
					return Optional.empty();
				}
			}
			
			if (fuzzSelType == FuzzingSelectionType.INTERROBOT_COMMS) {
				String k = cv.getKey();
				return confs.hasMessageKey(k);
			}
		}

		return Optional.empty();
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
		return op.fuzzTransformEvent(event);
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
