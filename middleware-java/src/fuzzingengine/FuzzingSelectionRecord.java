package fuzzingengine;

import fuzzingengine.operations.FuzzingOperation;

abstract public class FuzzingSelectionRecord {
	FuzzingOperation op;
	
	public FuzzingSelectionRecord(FuzzingOperation op) {
		this.op = op;
	}
}
