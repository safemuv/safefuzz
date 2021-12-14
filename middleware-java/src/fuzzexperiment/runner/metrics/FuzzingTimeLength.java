// protected region customHeaders on begin
package fuzzexperiment.runner.metrics;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;

import atlasdsl.Mission;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingTimeSpecification;
// protected region customHeaders end

public class FuzzingTimeLength extends OfflineMetric {
// protected region customFunction on begin
	public double countActiveFuzzingLength(String logDir) {
		Scanner reader;
		double totalLength = 0.0; 
		try {
			String filename = logDir + "/activeFuzzingCount.log";
				reader = new Scanner(new File(filename));
					while (reader.hasNextLine()) {
						String line = reader.nextLine();
						String[] fields = line.split(",");
						double time = Double.parseDouble(fields[0]);
						int val = Integer.parseInt(fields[1]);
						totalLength += val;
					}
					reader.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		return totalLength;
	}
		
	// protected region customFunction end

   public Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir, Mission mission) throws MetricComputeFailure {
		// Implement the metric here
		// protected region userCode on begin
		double totalLength = 0.0;
		boolean countActiveLength = true;
		
		for (FuzzingKeySelectionRecord r : recs) {
			if (r.isEnvironmental()) {
				// An environmental record is always counted as the full length
				// of the mission - since it always active
				System.out.println(this.getClass().getCanonicalName() + ": YAML variable found - adding full mission time length: ignoring static time length");
				double lenAdd = mission.getEndTime();
				totalLength += lenAdd;
			} else {
				FuzzingTimeSpecification ts = r.getTimeSpec();
				if (ts != null) {
	        		// This is only present for the statically defined length options.
	        		Optional<Double> slen_o = ts.getStaticLength();
	        		if (slen_o.isPresent()) {
	        			// If we have a static length, use it
	        			totalLength += slen_o.get();
	        		} else {
                                       // If not, use the logged active length
                                       // countActiveLength - A flag ensures this is only counted once
					if (countActiveLength) {
		        			// If not, use the logged active length
		        			// countActiveLength - A flag ensures this is only counted once
	        				totalLength += countActiveFuzzingLength(logDir);
						countActiveLength = false;
					}
	        		}
				}
			}
		}
        return totalLength;
    }  
// protected region userCode end
	
	public MetricDirection optimiseDirection() {
		// protected region userCode on begin
		return Metric.MetricDirection.LOWEST;
		// protected region userCode end
   } 
}
