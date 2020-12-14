package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import fuzzingengine.FuzzingConfig.FuzzingConfigRecord;
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
	
	public void addFuzzingOperation(String fuzzingKey, Optional<String> reflectionKey, Optional<String> component, FuzzingOperation op) {
		FuzzingConfigRecord fr = confs.new FuzzingConfigRecord(fuzzingKey, reflectionKey, component, Optional.empty(), op);
		confs.addRecord(fr);
	}

	// TODO: add something with subfields

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
	
	public String fuzzUDPEventKey(String key, String val) {
		if (fuzzSelType == FuzzingSelectionType.INTERROBOT_COMMS) {
			Optional<FuzzingOperation> op_o = confs.getOperationByKey(key);
			if (op_o.isPresent()) {
				FuzzingOperation op = op_o.get(); 
				return op.fuzzTransformEvent(val);
			} 
		}
		return val;
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
}
