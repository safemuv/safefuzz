// protected region customHeaders on begin
package fuzzexperiment.runner.metrics;
// protected region customHeaders end

import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Scanner;

import fuzzingengine.FuzzingKeySelectionRecord;

public class TrackSpeedViolations extends OfflineMetric {
	public Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir) throws MetricComputeFailure {
		// Implement the metric here
		// protected region userCode on begin
		int speedViolationsCount = 0;
		String filename = logDir + "/speedViolations.log";
		System.out.println("TrackSpeedViolations filename = " + filename);
		Scanner reader;
		try {
			reader = new Scanner(new File(filename));
			while (reader.hasNextLine()) {
				speedViolationsCount = speedViolationsCount + 1;
			}

			reader.close();
			System.out.println("speedViolationsCount = " + speedViolationsCount);
						
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return Double.valueOf(speedViolationsCount);	
		// protected region userCode end
	}
	
	public MetricDirection optimiseDirection() {
		// protected region userCode on begin
		return Metric.MetricDirection.HIGHEST;
		// protected region userCode end
	} 
}