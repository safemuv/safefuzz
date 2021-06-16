// protected region customHeaders on begin
package fuzzexperiment.runner.metrics;
// protected region customHeaders end

public class OutsideOfInnerRegionViolations extends OfflineMetric {
	public Double computeFromLogs(String logDir) throws MetricComputeFailure {
		// Implement the metric here
		// protected region userCode on begin
		return 0.0;
		// protected region userCode end
	}
}