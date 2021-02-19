package fuzzingengine.operations;

public class DelayFuzzingOperation extends QueueFuzzingOperation {
	private double time;
	public DelayFuzzingOperation(double time) {
		this.time = time;
	}
	
	public double enqueueTime() {
		return time;
	}
	
	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		String fields [] = s.split("\\|");
		System.out.println(fields[0]);
		if (fields[0].toUpperCase().equals("FIXED")) {
			double delay = Double.valueOf(fields[1]);
			return new DelayFuzzingOperation(delay);
		}
		
		throw new CreationFailed("Invalid parameter string " + s);
	}
}
