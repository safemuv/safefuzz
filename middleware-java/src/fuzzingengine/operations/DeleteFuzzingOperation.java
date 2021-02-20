package fuzzingengine.operations;

import java.util.Optional;
import java.util.Random;

public class DeleteFuzzingOperation extends EventFuzzingOperation {
	private double prob;
	private Random rng;
	
	public DeleteFuzzingOperation(double prob) {
		this.prob = prob;
		rng = new Random();
	}
	
	public double enqueueTime() {
		return 0.0;
	}
	
	public <E> Optional<E> fuzzTransformPotentialEvent(Optional<E> event) {
		// TODO: logging of dropping event
		if (rng.nextDouble() < prob) {
			System.out.println("DELETE: dropping event");
			return Optional.empty();
		} else {
			return event;
		}
	}
	
	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		String fields [] = s.split("\\|");
		System.out.println(fields[0]);
		if (fields[0].toUpperCase().equals("ALWAYS")) {
			return new DeleteFuzzingOperation(1.0);
		}
		
		if (fields[0].toUpperCase().equals("RANDOM")) {
			double prob = Double.valueOf(fields[1]);
			return new DeleteFuzzingOperation(prob);
		}
		
		throw new CreationFailed("Invalid parameter string " + s);
	}
}
