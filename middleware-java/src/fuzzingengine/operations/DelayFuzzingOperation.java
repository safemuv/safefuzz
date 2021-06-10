package fuzzingengine.operations;

import java.util.Random;
import fuzzingengine.UnitToDoubleLambda;

public class DelayFuzzingOperation extends QueueFuzzingOperation {
	private UnitToDoubleLambda getTime;
	private Random rng = new Random();
	
	public DelayFuzzingOperation(double time) {
		this.getTime = (() -> time);
	}
	
	public DelayFuzzingOperation(double minTime, double maxTime) {
		double delay = maxTime - minTime;
		this.getTime = (() -> minTime + (delay * rng.nextDouble()));
	}
	
	public double enqueueTime() {
		return getTime.op();
	}
	
	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		String fields [] = s.split("\\|");
		System.out.println(fields[0]);
		if (fields[0].toUpperCase().equals("FIXED")) {
			double delay = Double.valueOf(fields[1]);
			return new DelayFuzzingOperation(delay);
		}
		
		if (fields[0].toUpperCase().equals("RANDOM")) {
			double max = Double.valueOf(fields[1]);
			return new DelayFuzzingOperation(0.0, max);
		}
		
		throw new CreationFailed("Invalid parameter string " + s);
	}
}
