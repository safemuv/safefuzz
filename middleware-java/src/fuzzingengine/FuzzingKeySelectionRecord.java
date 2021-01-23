package fuzzingengine;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Pattern;

import atlasdsl.Robot;
import fuzzingengine.operations.FuzzingOperation;

public class FuzzingKeySelectionRecord extends FuzzingSelectionRecord {
	String key;
	Optional<String> reflectionKey;
	Optional<String> component;

	Optional<String> regex;
	Optional<Pattern> pattern;
	Optional<String> vehicleSelection;
	List<Robot> participants;
	HashMap<String,Boolean> participantsLookup = new HashMap<String,Boolean>();
	int groupNum;

	private void setupPattern() {
		if (regex.isPresent()) {
			this.pattern = Optional.of(Pattern.compile(regex.get()));
		} else {
			this.pattern = Optional.empty();
		}
	}
	
	private void setupParticipants() {
		for (Robot r : participants) {
			participantsLookup.put(r.getName(), true);
		}
	}
	
	public FuzzingKeySelectionRecord(String key, Optional<String> reflectionKey, Optional<String> component, Optional<String> regex,
			int groupNum, FuzzingOperation op, List<Robot> participants) {
		super(op);
		this.key = key;
		this.reflectionKey = reflectionKey;
		this.component = component;
		this.regex = regex;
		this.groupNum = groupNum;
		this.participants = participants;
		setupPattern();
		setupParticipants();
	}
	
	// for messages
	public FuzzingKeySelectionRecord(String key, Optional<String> reflectionKey, Optional<String> regex, int groupNum, FuzzingOperation op) {
		super(op);
		this.key = key;
		this.reflectionKey = reflectionKey;
		this.groupNum = groupNum;
		setupPattern();
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
	
	public Optional<Map.Entry<Pattern,Integer>> getPatternAndGroupNum() {
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
}
