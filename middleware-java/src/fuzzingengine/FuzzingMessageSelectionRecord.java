package fuzzingengine;

import java.util.List;

import atlasdsl.*;

import fuzzingengine.operations.FuzzingOperation;

public class FuzzingMessageSelectionRecord extends FuzzingSelectionRecord {
	String keyName;
	Message m;
	String robotNameFrom;
	String robotNameTo;

	private String lookupFirstComponentName(List<Component> cs) {
		Component c = cs.get(0);
		if (c instanceof Robot) {
			Robot r = (Robot)cs.get(0);
			return r.getName();
		} else {
			Computer co = (Computer)cs.get(0);
			return co.getName();
		}
	}
	
	private String getRobotNameMessageFrom() {
		List<Component> rnames = m.getFrom();
		return lookupFirstComponentName(rnames);
	}
	
	private String getRobotNameMessageTo() {
		List<Component> rnames = m.getTo();
		return lookupFirstComponentName(rnames);
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
