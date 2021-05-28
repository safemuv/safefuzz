package fuzzexperiment.runner;

import java.util.Optional;

public class RunRepeated extends ExptParams {
	private String resFileName;
	private int count = 0;
	private int limitCount;
	
	public RunRepeated(String resFileName, int limitCount) {
		this.resFileName = resFileName;
		this.count = 0;
		this.limitCount = limitCount;
	}

	public boolean completed() {
		return (count >= limitCount);
	}

	public void printState() {
		System.out.println("Run count = " + count);
	}

	public void advance() {
		count++;
	}

	public Optional<String> getNextFuzzingCSVFileName() {
		return Optional.of("fuzz-config.csv");
	}

}
