package fuzzexperiment.runner.metrics;

public abstract class OfflineMetric extends Metric {
	public abstract Object computeFromLogs(String logDir) throws MetricComputeFailure;
}
