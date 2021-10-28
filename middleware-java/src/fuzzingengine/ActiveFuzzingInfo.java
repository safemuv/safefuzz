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

	public ActiveFuzzingInfo(FuzzingKeySelectionRecord fr) {
		// TODO: implement this constructor
		this.key = fr.key;
		this.op = fr.op;
		this.pattern = fr.getPattern();
		
		if (fr.getGroupNum() == null) {
			this.subStructure = Optional.empty();
		} else {
			this.subStructure = Optional.of(fr.getGroupNum());
		}
	}

	public FuzzingOperation getOperation() {
		return op;
	}

	public Optional<Object> getJSONStructure() {
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
