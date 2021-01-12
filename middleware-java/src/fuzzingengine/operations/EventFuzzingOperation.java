package fuzzingengine.operations;

public abstract class EventFuzzingOperation extends FuzzingOperation {
	public boolean isEventBased() {
		return true;
	}
	
	public boolean isValueBased() {
		return false;
	}
	
	public abstract <E> E fuzzTransformEvent(E event);
}
