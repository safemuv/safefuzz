package fuzzingengine;

import atlasdsl.Message;
import fuzzingengine.operations.FuzzingOperation;

public class FuzzingMessageSelectionRecord extends FuzzingSelectionRecord {
	String keyName;
	Message m;

	public FuzzingMessageSelectionRecord(String keyName, Message m, FuzzingOperation op) {
		super(op);
		this.m = m;
		this.keyName = keyName;
	}
	
	public String getKey() {
		return keyName;
	}
}
