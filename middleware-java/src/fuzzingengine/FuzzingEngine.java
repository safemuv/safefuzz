package fuzzingengine;

import java.util.ArrayList;
import java.util.List;
import middleware.core.*;

public abstract class FuzzingEngine {
	public abstract <E> E fuzzTransformEvent(E event);
	
	// TODO: better way to map between the CARS variables and the messages
	// Can we encode the low-level CARS or translate in some way?
	private List<String> messageKeysToFuzz = new ArrayList<String>();
	private List<String> carsKeysToFuzz = new ArrayList<String>();
	
	public <E> boolean shouldFuzzCARSEvent(E event) {
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cv = (CARSVariableUpdate)event;
			String k = cv.getKey();
			return messageKeysToFuzz.contains(k) || carsKeysToFuzz.contains(k);
		} else {
			return false;
		}
	}
	
	public <E> boolean shouldReflectBackToCARS(E event) {
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cv = (CARSVariableUpdate)event;
			String k = cv.getKey();
			return messageKeysToFuzz.contains(k) || carsKeysToFuzz.contains(k);
		} else {
			return false;
		}
	}
}
