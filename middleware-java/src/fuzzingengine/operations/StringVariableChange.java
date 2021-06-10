package fuzzingengine.operations;

import java.util.Optional;
import java.util.Random;

import fuzzingengine.StringLambda;

public class StringVariableChange extends ValueFuzzingOperation {
	private final String defaultFixedChange = "JUNK";
	private String fixedChange = defaultFixedChange;
	private Optional<StringLambda> generateString = Optional.empty();
	
	public StringVariableChange(StringLambda generateDouble) {
		this.generateString = Optional.of(generateDouble);
	}
	
	public StringVariableChange(String fixedChange) {
		this.fixedChange = fixedChange;
	}
	
	public static StringVariableChange NewRandom(int sizeLimit, boolean alphaOnly) {
		Random r = new Random();
		
		StringLambda slam = (input -> {
			int len = r.nextInt(sizeLimit);
			StringBuilder sb = new StringBuilder();
			for (int i = 0; i < len; i++)
				if (alphaOnly) {
					sb.append(Character.toString((char)(65 + r.nextInt(26))));
				} else {
					sb.append(Character.toString((char)(r.nextInt(255))));
				}
			return sb.toString();
		});
		StringVariableChange op = new StringVariableChange(slam);
		return op;
	}

	public String getReplacement(String inValue) {
		return fixedChange;
	}

	public String fuzzTransformString(String input) {
		String changed = fixedChange;
		if (generateString.isPresent()) {
			StringLambda sl = generateString.get();
			changed = sl.op(input);
			System.out.println(changed);
		}
		return changed;
	}

	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		String fields [] = s.split("\\|");
		System.out.println(fields[0]);
		if (fields[0].toUpperCase().equals("NEWRANDOM")) {
			int maxLen = Integer.valueOf(fields[1]);
			return StringVariableChange.NewRandom(maxLen, false);
		}
		
		if (fields[0].toUpperCase().equals("RANDOMALPHA")) {
			int maxLen = Integer.valueOf(fields[1]);
			return StringVariableChange.NewRandom(maxLen, true);
		}

		throw new CreationFailed("Invalid parameter string " + s);
	}
}
