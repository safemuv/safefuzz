package fuzzingengine;

import java.util.HashMap;
import java.util.List;

import atlasdsl.Robot;
import fuzzingengine.operations.FuzzingOperation;

public class FuzzingComponentSelectionRecord extends FuzzingSelectionRecord {
	String componentName;
	List<Robot> participants;
	HashMap<String,Boolean> participantsLookup = new HashMap<String,Boolean>();
	
	private void setupParticipants() {
		for (Robot r : participants) {
			participantsLookup.put(r.getName(), true);
		}
	}
	
	public FuzzingComponentSelectionRecord(String componentName, FuzzingOperation op, List<Robot> participants) {
		super(op);
		this.componentName = componentName;
		this.participants = participants;
		setupParticipants();
	}
	
	public boolean hasVehicle(String vehicle) {
		return participantsLookup.containsKey(vehicle);
	}
}
