package fuzzingengine.operations;

import java.util.Optional;
import java.util.Random;

import fuzzingengine.DoubleLambda;

public class DoubleVariableChange extends ValueFuzzingOperation {
	private final double defaultFixedChange = 0.0;
	private String fixedChange = Double.toString(defaultFixedChange);
	private Optional<DoubleLambda> generateDouble = Optional.empty();
	
	public DoubleVariableChange(DoubleLambda generateDouble) {
		this.generateDouble = Optional.of(generateDouble);
	}
	
	public DoubleVariableChange(double fixedChange) {
		this.fixedChange = Double.toString(fixedChange);
	}
	
	public static DoubleVariableChange Random(double lower, double upper) {
		Random r = new Random();
		double diff = upper - lower;
		DoubleVariableChange op = new DoubleVariableChange(input -> lower + (diff * r.nextDouble()));
		return op;
	}
	
	public static DoubleVariableChange RandomOffset(double lower, double upper) {
		Random r = new Random();
		double diff = upper - lower;
		DoubleVariableChange op = new DoubleVariableChange(input -> input + (lower + (diff * r.nextDouble())));
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
}
