package exptrunner;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import atlasdsl.*;


public class FuzzingSelection {
	// Constraints should be generated via EGL to ensure that the generated fuzzing selection etc will
	// conform to the model - in this file?

	private String affectedKey;
	
	// messageRef is null if this is a key-based entity
	private Optional<Message> messageRef;
	private List<String> participants;
	private String operation;
	private List<String> opParams;
	private double startTime;
	private double endTime;
	private int regexNum = 0;
	
	private List<String> duplicateArray(List<String> newParticipants) {
		participants = new ArrayList<String>();
		for (String p : newParticipants) {
			participants.add(new String(p));
		}
		return participants;
	}
	
	public FuzzingSelection(FuzzingSelection other) {
		this.affectedKey = other.affectedKey;
		this.messageRef = other.messageRef;
		this.participants = duplicateArray(other.participants);
		this.operation = other.operation;
		this.opParams = duplicateArray(other.opParams);
		this.startTime = other.startTime;
		this.endTime = other.endTime;
	}
	
	public FuzzingSelection(String affectedKey, List<String> participants, String operation, List<String> opParams) {
		this.affectedKey = affectedKey;
		this.participants = duplicateArray(participants);
		this.operation = operation;
		this.opParams = opParams;
	}

	public boolean isActive() {
		return true;
	}

	public double getEndTime() {
		return endTime;
	}

	public double getStartTime() {
		return startTime;
	}
	
	private List<String> generateMessageCSVLine() {
		List<String> str = new ArrayList<String>();
		str.add("MESSAGE");
		str.add(messageRef.get().getName());
		str.add(String.valueOf(startTime));
		str.add(String.valueOf(endTime));
		str.add(affectedKey);
		str.add(String.valueOf(regexNum));
		str.add(String.valueOf(operation));
		str.add(generateOpParams());
		return str;
	}
	
	private String generateOpParams() {
		return String.join("|", String.valueOf(opParams));
	}
	
	private List<String> generateKeyCSVLine() {
		List<String> str = new ArrayList<String>();
		str.add("KEY");
		str.add(affectedKey);
		str.add(String.valueOf(startTime));
		str.add(String.valueOf(endTime));
		str.add(String.join("|", participants));
		str.add(String.valueOf(regexNum));
		str.add(String.valueOf(operation));
		str.add(generateOpParams());
		return str;
	}
	
	public String generateCSVLine() {
		List<String> res;
		if (messageRef.isPresent()) {
			res = generateMessageCSVLine();
		} else {
			res = generateKeyCSVLine();
		}
		return String.join(",", res);
	}
}
