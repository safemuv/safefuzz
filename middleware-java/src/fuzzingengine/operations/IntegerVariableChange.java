package fuzzingengine.operations;

import java.util.Optional;
import java.util.Random;
import fuzzingengine.IntLambda;

public class IntegerVariableChange extends ValueFuzzingOperation {
	private final int defaultFixedChange = 0;
	private String fixedChange = Integer.toString(defaultFixedChange);
	private Optional<IntLambda> generateInt = Optional.empty();
	
	public IntegerVariableChange(IntLambda generateInt) {
		this.generateInt = Optional.of(generateInt);
	}
	
	public IntegerVariableChange(int fixedChange) {
		this.fixedChange = Integer.toString(fixedChange);
	}
	
	public static IntegerVariableChange Random(int lower, int upper) {
		Random r = new Random();
		int diff = upper - lower;
		IntegerVariableChange op = new IntegerVariableChange(input -> lower + (int)Math.floor(diff * r.nextDouble()));
		return op;
	}
	
	public static IntegerVariableChange RandomOffset(int lower, int upper) {
		Random r = new Random();
		int diff = upper - lower;
		IntegerVariableChange op = new IntegerVariableChange(input -> input + (int)Math.floor(lower + (diff * r.nextDouble())));
		return op;
	}

	public String getReplacement(String inValue) {
		return fixedChange;
	}

	public String fuzzTransformString(String input) {
		String changed = fixedChange;
		if (generateInt.isPresent()) {
			IntLambda dl = generateInt.get();
			int d = Integer.valueOf(input);
			changed = Integer.toString(dl.op(d));
		}
		return changed;
	}
	
	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		String fields [] = s.split("\\|");
		System.out.println(fields[0]);
		if (fields[0].toUpperCase().equals("RANDOM")) {
			int l = Integer.valueOf(fields[1]);
			int r = Integer.valueOf(fields[2]);
			return IntegerVariableChange.Random(l,r);
		}
		
		throw new CreationFailed("Invalid parameter string " + s);
	}
}
