package fuzzingengine.operations;

import java.util.Optional;

public class NullFuzzingOperation extends EventFuzzingOperation {
	
	public <E> Optional<E> fuzzTransformPotentialEvent(Optional<E> event) {
		return event;
	}
}
