package fuzzingengine;

import middleware.core.MOOSVariableUpdate;

public abstract class FuzzingEngine {
	public abstract MOOSVariableUpdate fuzzMessage(MOOSVariableUpdate mup);
}
