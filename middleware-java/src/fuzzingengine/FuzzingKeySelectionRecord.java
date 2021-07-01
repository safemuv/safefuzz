package fuzzingengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

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
	List<Object> params;

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
			Object groupNum, FuzzingOperation op, List<String> participants, double startTime, double endTime) {
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
	
	public double getTimeLength() {
		return endTime - startTime;
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
	
	public void setParams(List<Object> params) {
		this.params = params;
	}
	
	private String paramsAsString() {
		List<String> strList = params.stream().map(e -> e.toString()).collect(Collectors.toList());
		// Merge the specificOpParams with pipes
		String paramStr = String.join("|", strList);
		return paramStr;
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
		// Assume that always empty vehicles are just general simulator topics. They return true
		return (vehicle == "") || participantsLookup.containsKey(vehicle);
	}

	public FuzzingSelectionRecord dup() {
		List<String> newParticipants = new ArrayList<String>();
		for (String p : this.participants) {
			newParticipants.add(new String(p));
		}
		
		FuzzingKeySelectionRecord k = new FuzzingKeySelectionRecord(key, reflectionKey,  component, regex,
				groupNum, op, newParticipants, startTime, endTime);
		// As long as setParams is called later on uniquely generated parameters, should be OK!
		k.setParams(params);
		return k;
	}

	private String operationName() {
		return op.getClass().getSimpleName();
	}

	public String generateCSVLine() {
        List<String> str = new ArrayList<String>();
        str.add("KEY");
        str.add(key);
        str.add(String.valueOf(startTime));
        str.add(String.valueOf(endTime));
        str.add(String.join("|", participants));
        str.add(String.valueOf(groupNum));
        str.add(operationName());
        str.add(paramsAsString());
        return String.join(",", str);
	}

	public void checkConstraints() {

		
	}
	
	public Object getGroupNum() {
		return groupNum;
	}

	public void setParticipants(List<String> newParticipants) {
		this.participants = newParticipants;
	}
}
