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
	
	public void addFuzzKey(String in) {
		System.out.println("addFuzzKey = " + in + ",length=" + in.length());
		messageKeysToFuzz.add(in);
	}
	
	public void addFuzzKey(String in, String out) {
		carsKeysReflection.put(in,out);
	}
	
	public void clearKeys() {
		carsKeysReflection.clear();
		messageKeysToFuzz.clear();
	}
	
	public <E> boolean shouldFuzzCARSEvent(E event) {
		System.out.println("shouldFuzzCARSEvent");
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cv = (CARSVariableUpdate)event;
			String k = cv.getKey();
			if (k.equals("DB_UPTIME")) {
				System.out.println("matches");
				System.out.println(messageKeysToFuzz);
			}
			System.out.println("k = " + k + ",length=" + k.length());
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
