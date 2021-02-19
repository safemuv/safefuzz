package fuzzingengine.operations;

import java.util.Optional;

public class QueueFuzzingOperation extends EventFuzzingOperation {
	public <E> Optional<E> fuzzTransformPotentialEvent(Optional<E> event) {
		return event;
	}
	
	public boolean shouldEnqueue() { 
		return true;
	}
}
