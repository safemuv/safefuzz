// protected region customHeaders on begin
package fuzzexperiment.runner.metrics;
import java.util.List;
import java.util.Optional;

import fuzzexperiment.runner.metrics.Metric.MetricDirection;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingTimeSpecification;
// protected region customHeaders end

public class FuzzingTimeLength extends OfflineMetric {
	public Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir) throws MetricComputeFailure {
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
        			
        			if (r.getKey().toLowerCase().contains("yaml")) {
        				System.out.println(this.getClass().getCanonicalName() + ": YAML variable found - ignoring static time length of " + slenAdd);
        				slenAdd = 0;
        			}
        			totalLength += slenAdd;
        		} else {
       				System.out.println(this.getClass().getCanonicalName() + ": variable " + r.getKey() + " no static time length - normal if using condition-based fuzzing");
        		}
        	}
        }

        // Since it is a metric designed to be minimsed, set it as negative
        return totalLength;
		// protected region userCode end
	}
	
	public MetricDirection optimiseDirection() {
		// protected region userCode on begin
		return Metric.MetricDirection.LOWEST;
		// protected region userCode end
	} 
}