package fuzzingengine;

import middleware.core.CARSVariableUpdate;

public class ExternalFuzzingEngine extends FuzzingEngine {

	public ExternalFuzzingEngine() {
		// Load the external fuzzer process
		// Specify the fuzzing capability from the DSL
	}

	public <E> E fuzzTransformEvent(E event) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public <E> boolean shouldFuzzCARSEvent(E event) {
		return false;
	}

	@Override
	public <E> boolean shouldReflectBackToCARS(E event) {
		return false;
	}
}
