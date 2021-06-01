package fuzzexperiment.runner.metrics;

import java.util.Scanner;

public class OutsideOfOuterRegionViolations extends OfflineMetric {
	public Object computeFromLogs(String logDir) throws MetricComputeFailure {
		String filename = logDir + "/goalLog.log";
		Scanner reader = new Scanner(filename);
		int outsideRegionViolations = 0;
		
		while (reader.hasNextLine()) {
			String line = reader.nextLine();
			String[] fields = line.split(",");
			String goalClass = fields[0];
			

			if (goalClass.equals("atlasdsl.StayInRegion")) {
				int count = Integer.parseInt(fields[1]);
				outsideRegionViolations = Math.max(count, outsideRegionViolations);
			}
		}
		
		reader.close();
		return outsideRegionViolations;
	}
}
