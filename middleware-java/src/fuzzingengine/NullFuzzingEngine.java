package fuzzingengine;

import atlasdsl.Mission;

public class NullFuzzingEngine extends FuzzingEngine {
	public NullFuzzingEngine(Mission m) {
		super(m);
	}

	public <E> E fuzzTransformEvent(E event) {
		return event;
	}
}