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
	public Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir, Mission mission) throws MetricComputeFailure {
		// Implement the metric here
		// protected region userCode on begin
		double totalLength = 0.0;
        for (FuzzingKeySelectionRecord r : recs) {
        	FuzzingTimeSpecification ts = r.getTimeSpec();
        	if (ts != null) {
        		// This is only present for the statically defined length options.
        		Optional<Double> slen_o = ts.getStaticLength();
        		if (slen_o.isPresent()) {
        			double slenAdd = slen_o.get();
        			if (r.isEnvironmental()) {
        				System.out.println(this.getClass().getCanonicalName() + ": YAML variable found - adding full mission time length: ignoring static time length of " + slenAdd);
        				slenAdd = mission.getEndTime() * mission.getRobotCount();
        			}
        			totalLength += slenAdd;
        		} else {
        			// There are no time lengths defined here, we need to define it simply
        			Scanner reader;
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
	       				return totalLength;
					} catch (FileNotFoundException e) {
						e.printStackTrace();
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