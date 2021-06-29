// protected region customHeaders on begin
package fuzzexperiment.runner.metrics;
import java.util.List;

import fuzzingengine.FuzzingKeySelectionRecord;
// protected region customHeaders end

public class FuzzingTimeLength extends OfflineMetric {
	public Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir) throws MetricComputeFailure {
		// Implement the metric here
		// protected region userCode on begin
		double totalLength = 0.0;
        for (FuzzingKeySelectionRecord r : recs) {
            totalLength += r.getTimeLength();
        }

        // Since it is a metric designed to be minimsed, set it as negative
        return -totalLength;
		// protected region userCode end
	}
}