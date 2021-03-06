package fuzzexperiment.runner.metrics;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import atlasdsl.Mission;

// TODO: factor these out and make MetricHandler an interface
public class FakeMetricHandler extends MetricHandler {
	
	private Random rngforMetrics = new Random();
	
	public FakeMetricHandler(Mission mission, List<OfflineMetric> metrics, String resFileName) throws IOException {
		super(mission, metrics,resFileName);
	}
	
	public Map<Metric, Double> computeAllOffline(String logDir) throws MetricComputeFailure {
		Map<Metric, Double> results = new HashMap<Metric, Double>();
		for (OfflineMetric m : metrics) {
			Double res = rngforMetrics.nextDouble();
			results.put(m, res);
		}
		return results;
	}
}
