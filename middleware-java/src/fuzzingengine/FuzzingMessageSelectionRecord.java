package fuzzingengine;

import java.util.ArrayList;
import java.util.List;

import atlasdsl.*;

import fuzzingengine.operations.FuzzingOperation;

public class FuzzingMessageSelectionRecord extends FuzzingSelectionRecord {
	String keyName;
	Message m;
	String robotNameFrom;
	String robotNameTo;
	int regexNum = 0;

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

	public FuzzingSelectionRecord dup() {
		// TODO Auto-generated method stub
		return null;
	}

	public String generateCSVLine() {
        List<String> str = new ArrayList<String>();
        str.add("MESSAGE");
        str.add(m.getName());
        str.add(String.valueOf(startTime));
        str.add(String.valueOf(endTime));
        str.add(keyName);
        str.add(String.valueOf(regexNum));
        str.add(String.valueOf(op));
        str.add(generateOpParams());
        return String.join(",", str);
	}

	public void checkConstraints() {
		// TODO Auto-generated method stub
	}
}
