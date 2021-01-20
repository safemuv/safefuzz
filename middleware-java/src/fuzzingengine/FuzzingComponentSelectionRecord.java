package fuzzingengine;

import fuzzingengine.operations.FuzzingOperation;

public class FuzzingComponentSelectionRecord extends FuzzingSelectionRecord {
	String componentName;
	
	public FuzzingComponentSelectionRecord(String componentName, FuzzingOperation op) {
		super(op);
		this.componentName = componentName;
	}
}
