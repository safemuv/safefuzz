package fuzzingengine.operations;

public class NullFuzzingOperation extends FuzzingOperation {
	public <E> E fuzzTransformEvent(E event) {
		return event;
	}
}
