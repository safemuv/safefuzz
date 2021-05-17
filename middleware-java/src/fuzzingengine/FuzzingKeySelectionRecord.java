package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Pattern;

import fuzzingengine.operations.FuzzingOperation;

public class FuzzingKeySelectionRecord extends FuzzingSelectionRecord {
	String key;
	Optional<String> reflectionKey;
	Optional<String> component = Optional.empty();

	Optional<String> regex;
	Optional<Pattern> pattern;
	Optional<String> vehicleSelection;
	List<String> participants = new ArrayList<String>();
	HashMap<String,Boolean> participantsLookup = new HashMap<String,Boolean>();
	Object groupNum;

	private void setupPattern() {
		if (regex.isPresent()) {
			this.pattern = Optional.of(Pattern.compile(regex.get()));
		} else {
			this.pattern = Optional.empty();
		}
	}
	
	private void setupParticipants() {
		for (String r : participants) {
			participantsLookup.put(r, true);
		}
	}
	
	public FuzzingKeySelectionRecord(String key, Optional<String> reflectionKey, Optional<String> component, Optional<String> regex,
			int groupNum, FuzzingOperation op, List<String> participants, double startTime, double endTime) {
		super(op);
		this.key = key;
		this.reflectionKey = reflectionKey;
		this.component = component;
		this.regex = regex;
		this.groupNum = groupNum;
		this.participants = participants;
		this.startTime = startTime;
		this.endTime = endTime;
		this.op = op;
		setupPattern();
		setupParticipants();
	}
	
	// for messages
	public FuzzingKeySelectionRecord(String key, Optional<String> reflectionKey, Optional<String> regex, int groupNum, FuzzingOperation op) {
		super(op);
		this.key = key;
		this.reflectionKey = reflectionKey;
		this.component = Optional.empty();
		this.groupNum = groupNum;
		this.regex = regex;
		setupPattern();
	}
	
	public void addParticipant(String r) {
		participants.add(r);
		participantsLookup.put(r,true);
	}

	public String getKey() {
		return key;
	}

	public Optional<String> getReflectionKey() {
		return reflectionKey;
	}

	public boolean hasComponent() {
		return component.isPresent();
	}
	
	public String getComponent() {
		return component.get();
	}

	public FuzzingOperation getOperation() {
		return op;
	}
	
	public Optional<Pattern> getPattern() {
		return pattern;
	}
	
	public Optional<Map.Entry<Pattern,Object>> getPatternAndGroupStructure() {
		if (pattern.isPresent()) {
			Pattern p = pattern.get();
			return Optional.of(Map.entry(p, groupNum));
		} else {
			return Optional.empty();
		}
	}

	public boolean hasVehicle(String vehicle) {
		return participantsLookup.containsKey(vehicle);
	}

	@Override
	public FuzzingSelectionRecord dup() {
		// TODO Auto-generated method stub
		return null;
	}

	public String generateCSVLine() {
        List<String> str = new ArrayList<String>();
        str.add("KEY");
        str.add(key);
        str.add(String.valueOf(startTime));
        
        str.add(String.valueOf(endTime));
        str.add(String.join("|", participants));
        str.add(String.valueOf(groupNum));
        str.add(String.valueOf(op));
        str.add(generateOpParams());
        return String.join(",", str);
	}

	@Override
	public void checkConstraints() {
		// TODO Auto-generated method stub
		
	}
}
