package fuzzexperiment.runner.metrics;

import java.util.Scanner;

public class OutsideOfInnerRegionViolations extends OfflineMetric {
	public Object computeFromLogs(String logDir) throws MetricComputeFailure {
		String filename = logDir + "/goalLog.log";
		Scanner reader = new Scanner(filename);
		int innerRegionViolations = 0;
		
		// Count violations from Pedro's topic when supplied
		
		reader.close();
		return innerRegionViolations;
	}
}
