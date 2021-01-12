package fuzzingengine.operations;

public class NullFuzzingOperation extends EventFuzzingOperation {
	public <E> E fuzzTransformEvent(E event) {
		return event;
	}
}
