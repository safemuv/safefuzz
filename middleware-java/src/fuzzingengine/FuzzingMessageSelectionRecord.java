package fuzzingengine;

import java.util.List;

import atlasdsl.*;

import fuzzingengine.operations.FuzzingOperation;

public class FuzzingMessageSelectionRecord extends FuzzingSelectionRecord {
	String keyName;
	Message m;
	String robotNameFrom;
	String robotNameTo;

	private String getRobotNameMessageFrom() {
		List<Component> rnames = m.getFrom();
		Component c = rnames.get(0);
		Robot r = (Robot)rnames.get(0);
		return r.getName();
	}
	
	private String getRobotNameMessageTo() {
		List<Component> rnames = m.getTo();
		Component c = rnames.get(0);
		Robot r = (Robot)rnames.get(0);
		return r.getName();
	}
	
	public FuzzingMessageSelectionRecord(String keyName, Message m, FuzzingOperation op) {
		super(op);
		this.m = m;
		this.robotNameFrom = getRobotNameMessageFrom();
		this.robotNameTo = getRobotNameMessageTo();
		this.keyName = keyName;
	}
	
	public String getKey() {
		return keyName;
	}

	public FuzzingOperation getOperation() {
		return op;
	}
	
	public String getRobotFrom() {
		return robotNameFrom;
	}
	
	public String getRobotTo() {
		return robotNameTo;
	}
}
