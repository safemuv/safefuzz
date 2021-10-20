package fuzzexperiment.runner.jmetal;

import java.io.FileWriter;
import java.io.IOException;

import org.uma.jmetal.operator.crossover.CrossoverOperator;

public abstract class FuzzingCrossoverOperation implements CrossoverOperator<FuzzingSelectionsSolution> {
	private static final long serialVersionUID = 1L;
	protected FileWriter crossoverLog;
	
	FuzzingCrossoverOperation(String crossoverLogFileName) throws IOException {
		this.crossoverLog = new FileWriter(crossoverLogFileName);
	}
	
	protected void logWithoutException(String s) {
		try {
			crossoverLog.write(s);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void closeLog() throws IOException {
		crossoverLog.close();	
	}
}
