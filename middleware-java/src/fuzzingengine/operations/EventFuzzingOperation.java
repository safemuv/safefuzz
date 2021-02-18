package fuzzingengine.operations;

import java.util.Optional;

public abstract class EventFuzzingOperation extends FuzzingOperation {
	public boolean isEventBased() {
		return true;
	}
	
	public boolean isValueBased() {
		return false;
	}
	
	public abstract <E> Optional<E> fuzzTransformPotentialEvent(Optional<E> event);
}
