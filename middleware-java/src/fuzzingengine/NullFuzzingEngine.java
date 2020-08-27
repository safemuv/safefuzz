package fuzzingengine;

import middleware.core.CARSVariableUpdate;

public class NullFuzzingEngine extends FuzzingEngine {
	public CARSVariableUpdate fuzzMessage(CARSVariableUpdate mup) {
		return mup;
	}

	public <E> E fuzzTransformEvent(E event) {
		return event;
	}
}