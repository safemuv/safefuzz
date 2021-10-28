package fuzzingengine;

import java.util.Map.Entry;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Pattern;

import fuzzingengine.operations.FuzzingOperation;

// This class acts as a temporary record to store information on the
// fuzzing which is currently active.
public class ActiveFuzzingInfo {
	public String key;
	public FuzzingOperation op;
	public Optional<Object> subStructure;
	public Optional<Pattern> pattern;
	
	ActiveFuzzingInfo(String key, FuzzingOperation op, Optional<Object> jsonStructure) {
		this.key = key;
		this.op = op;
	}

	public ActiveFuzzingInfo(FuzzingKeySelectionRecord fr) {
		// TODO: implement this constructor
	}

	public FuzzingOperation getOperation() {
		return op;
	}

	public Optional<Object> getJSONStructure(String key2) {
		return subStructure;
	}

	public Optional<Entry<Pattern, Object>> getPatternAndGroupStructure() {
		if (pattern.isPresent()) {
			Pattern p = pattern.get();
			return Optional.of(Map.entry(p, subStructure));
		} else {
			return Optional.empty();
		}
	}
}
