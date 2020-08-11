package fuzzingengine;

import middleware.core.MOOSVariableUpdate;

public class NullFuzzingEngine extends FuzzingEngine {
	public MOOSVariableUpdate fuzzMessage(MOOSVariableUpdate mup) {
		return mup;
	}
}