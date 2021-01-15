package fuzzingengine.operations;

import java.util.Optional;
import java.util.Random;

import fuzzingengine.DoubleLambda;

public class StringVariableChange extends ValueFuzzingOperation {
	private final double defaultFixedChange = 0.0;
	private String fixedChange = Double.toString(defaultFixedChange);
	private Optional<DoubleLambda> generateDouble = Optional.empty();
	
	public StringVariableChange(DoubleLambda generateDouble) {
		this.generateDouble = Optional.of(generateDouble);
	}
	
	public StringVariableChange(double fixedChange) {
		this.fixedChange = Double.toString(fixedChange);
	}
	
	public static StringVariableChange Random(double lower, double upper) {
		Random r = new Random();
		double diff = upper - lower;
		StringVariableChange op = new StringVariableChange(input -> lower + (diff * r.nextDouble()));
		return op;
	}
	
	public static StringVariableChange RandomOffset(double lower, double upper) {
		Random r = new Random();
		double diff = upper - lower;
		StringVariableChange op = new StringVariableChange(input -> input + (lower + (diff * r.nextDouble())));
		return op;
	}

	public String getReplacement(String inValue) {
		return fixedChange;
	}

	public String fuzzTransformString(String input) {
		String changed = fixedChange;
		if (generateDouble.isPresent()) {
			DoubleLambda dl = generateDouble.get();
			double d = Double.valueOf(input);
			changed = Double.toString(dl.op(d));
		}
		return changed;
	}

	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		throw new CreationFailed("StringVariableChange unimplemented");
	}
}
