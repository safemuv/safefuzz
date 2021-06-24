// protected region customHeaders on begin
package fuzzexperiment.runner.metrics;
// protected region customHeaders end

import java.util.List;

import fuzzingengine.FuzzingKeySelectionRecord;

public class AvoidanceViolations extends OfflineMetric {
	public Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir) throws MetricComputeFailure {
		// Implement the metric here
		// protected region userCode on begin
		return 0.0;
		// protected region userCode end
	}
}