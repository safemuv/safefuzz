package fuzzingengine;

import fuzzingengine.operations.FuzzingOperation;

abstract public class FuzzingSelectionRecord {
	protected FuzzingOperation op;
	
	public FuzzingSelectionRecord(FuzzingOperation op) {
		this.op = op;
	}
	
	protected FuzzingOperation getOperation() {
		return op;
	}
}
