package fuzzingengine;

import java.util.HashMap;
import java.util.List;

import atlasdsl.Robot;
import fuzzingengine.operations.FuzzingOperation;

public class FuzzingComponentSelectionRecord extends FuzzingSelectionRecord {
	String componentName;
	List<String> participants;
	HashMap<String,Boolean> participantsLookup = new HashMap<String,Boolean>();
	
	private void setupParticipants() {
		for (String n : participants) {
			participantsLookup.put(n, true);
		}
	}
	
	public FuzzingComponentSelectionRecord(String componentName, FuzzingOperation op, List<String> participants) {
		super(op);
		this.componentName = componentName;
		this.participants = participants;
		setupParticipants();
	}
	
	public boolean hasVehicle(String vehicle) {
		return participantsLookup.containsKey(vehicle);
	}

	@Override
	public FuzzingSelectionRecord dup() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public String generateCSVLine() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void checkConstraints() {
		// TODO Auto-generated method stub
		
	}
}
