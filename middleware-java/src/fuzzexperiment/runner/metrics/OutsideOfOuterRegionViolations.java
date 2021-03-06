// protected region customHeaders on begin
package fuzzexperiment.runner.metrics;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Scanner;

import atlasdsl.Mission;
import fuzzexperiment.runner.metrics.Metric.MetricDirection;
import fuzzingengine.FuzzingKeySelectionRecord;
// protected region customHeaders end

public class OutsideOfOuterRegionViolations extends OfflineMetric {
// protected region customFunction on begin
// protected region customFunction end

   public Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir, Mission mission) throws MetricComputeFailure {
		// Implement the metric here
		// protected region userCode on begin
		int outsideRegionViolations = 0;
		String filename = logDir + "/goalLog.log";
		System.out.println("OutsideOfOuterRegionViolations filename = " + filename);
		Scanner reader;
		try {
			reader = new Scanner(new File(filename));
			while (reader.hasNextLine()) {
				String line = reader.nextLine();
				String[] fields = line.split(",");
				String goalClass = fields[0];

				//System.out.println("goalClass = " + goalClass);
				if (goalClass.equals("atlasdsl.StayInRegion")) {
					int count = Integer.parseInt(fields[1]);
					outsideRegionViolations = Math.max(count, outsideRegionViolations);
				}
			}

			reader.close();
			System.out.println("outsideRegionViolations = " + outsideRegionViolations);
						
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return Double.valueOf(outsideRegionViolations);	
		// protected region userCode end
	}
	
	public MetricDirection optimiseDirection() {
		// protected region userCode on begin
		return Metric.MetricDirection.HIGHEST;
		// protected region userCode end
   } 
}
