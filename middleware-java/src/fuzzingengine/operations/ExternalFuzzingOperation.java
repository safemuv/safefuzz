package fuzzingengine.operations;

import java.util.Optional;

public class ExternalFuzzingOperation extends EventFuzzingOperation {

	public ExternalFuzzingOperation() {
		// Load the external fuzzer process
		// Specify the fuzzing capability from the DSL
	}

	public <E> Optional<E> fuzzTransformPotentialEvent(Optional<E> event) {
		// TODO Auto-generated method stub
		return event;
	}

	public static FuzzingOperation createFromParamString(String s) {
		return null;
	}
}
