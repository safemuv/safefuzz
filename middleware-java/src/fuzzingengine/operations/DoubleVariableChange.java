package fuzzingengine.operations;

import java.util.Optional;
import java.util.Random;

import fuzzingengine.DoubleLambda;
import middleware.logging.ATLASLog;

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
	
	public static DoubleVariableChange Random(double lower, double upper, long seed) {
		Random r = new Random(seed);
		System.out.println("DoubleVariableChange - Creating random generator with seed " + seed);
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
	
	public static DoubleVariableChange RandomOffset(double lower, double upper, long seed) {
		Random r = new Random(seed);
		System.out.println("DoubleVariableChange - Creating random generator with seed " + seed);
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
			ATLASLog.logFuzzing("DoubleVariableChange - input = " + input + ",output=" + changed);
		}
		return changed;
	}

	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		String fields [] = s.split("\\|");
		System.out.println(fields[0]);
		if (fields[0].toUpperCase().equals("RANDOM")) {
			double l = Double.valueOf(fields[1]);
			double r = Double.valueOf(fields[2]);
			return DoubleVariableChange.Random(l,r);
		}
		
		if (fields[0].toUpperCase().equals("RANDOMOFFSET")) {
			double l = Double.valueOf(fields[1]);
			double r = Double.valueOf(fields[2]);
			return DoubleVariableChange.RandomOffset(l,r);
		}
		
		if (fields[0].toUpperCase().equals("RANDOMWITHSEED")) {
			double l = Double.valueOf(fields[1]);
			double r = Double.valueOf(fields[2]);
			long seed = Long.valueOf(fields[3]);
			return DoubleVariableChange.Random(l,r,seed);
		}
		
		if (fields[0].toUpperCase().equals("RANDOMOFFSETWITHSEED")) {
			double l = Double.valueOf(fields[1]);
			double r = Double.valueOf(fields[2]);
			long seed = Long.valueOf(fields[3]);
			return DoubleVariableChange.RandomOffset(l,r,seed);
		}
		
		throw new CreationFailed("Invalid parameter string " + s);
	}
}
