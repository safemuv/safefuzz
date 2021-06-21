// protected region customHeaders on begin
package fuzzexperiment.runner.metrics;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
// protected region customHeaders end

public class SpeedViolationsCount extends OfflineMetric {
	public Double computeFromLogs(String logDir) throws MetricComputeFailure {
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
			System.out.println("SpeedViolationsCount: filename " + filename + " not found: this is normal if there are no speed violations");
		}
		
		return Double.valueOf(speedViolationsCount);	
		// protected region userCode end
	}
}