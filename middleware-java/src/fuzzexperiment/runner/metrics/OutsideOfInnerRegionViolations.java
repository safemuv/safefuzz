// protected region customHeaders on begin
package fuzzexperiment.runner.metrics;
import java.util.List;
import fuzzingengine.FuzzingKeySelectionRecord;
// protected region customHeaders end

public class OutsideOfInnerRegionViolations extends OfflineMetric {
	public Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir) throws MetricComputeFailure {
		// Implement the metric here
		// protected region userCode on begin
		return 0.0;
		// protected region userCode end
	}
}