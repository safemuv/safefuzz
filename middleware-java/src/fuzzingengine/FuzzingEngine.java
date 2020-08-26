package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Optional;

import middleware.core.*;

public abstract class FuzzingEngine {
	public abstract <E> E fuzzTransformEvent(E event);
	
	// TODO: better way to map between the CARS variables and the messages
	// Can we encode the low-level CARS or translate in some way?
	private List<String> messageKeysToFuzz = new ArrayList<String>();
	
	// This contains the 
	private HashMap<String,String> carsKeysReflection = new LinkedHashMap<String,String>();
	
	public <E> boolean shouldFuzzCARSEvent(E event) {
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cv = (CARSVariableUpdate)event;
			String k = cv.getKey();
			return messageKeysToFuzz.contains(k) || carsKeysReflection.containsKey(k);
		} else {
			return false;
		}
	}
	
	public <E> Optional<String> shouldReflectBackToCARS(E event) {
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cv = (CARSVariableUpdate)event;
			String k = cv.getKey();
			if (carsKeysReflection.containsKey(k)) {
				return Optional.of(carsKeysReflection.get(k));
			}
		}
		return Optional.empty();
	}
}
