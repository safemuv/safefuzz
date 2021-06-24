package fuzzexperiment.runner.metrics;

import java.util.List;
import fuzzingengine.FuzzingKeySelectionRecord;

public abstract class OfflineMetric extends Metric {
	public abstract Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir) throws MetricComputeFailure;
}
