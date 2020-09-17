package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import carsspecific.moos.carsqueue.PShareEvent;
import fuzzingengine.operations.FuzzingOperation;
import middleware.core.*;
import middleware.logging.ATLASLog;

public class FuzzingEngine {
	FuzzingConfig confs = new FuzzingConfig();
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
}
