package fuzzexperiment.runner.metrics;

import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import exptrunner.jmetal.InvalidMetrics;
import exptrunner.metrics.Metrics;

public class MetricHandler {
	// TODO: how to supply content to the metrics? - e.g. structure of the solution?
	private List<OfflineMetric> metrics;
	private FileWriter resFile;
	private String resFileName;
	
	public MetricHandler(List<OfflineMetric> metrics) {
		this.metrics = metrics;
	}
	
	public void printHeader(String resFileName) throws IOException {
		resFile = new FileWriter(resFileName);
		this.resFileName = resFileName;
		for (OfflineMetric m : metrics) {
			resFile.write(m.getClass().getSimpleName());
		}
	}
	
	public void printMetrics(Map<Metric,Object> metricRes) throws IOException {
			System.out.println("Writing results to result file: " + resFileName);
			for (OfflineMetric m : metrics) {
				Object val = metricRes.get(m);
				System.out.println(m + "=" + val);
				resFile.write(val.toString() + ",");
			}			
			resFile.write("\n");
			resFile.flush();	
	}
	
	public Map<Metric,Object> computeAllOffline(String logDir) throws MetricComputeFailure {
		Map<Metric,Object> results = new HashMap<Metric,Object>();
		for (OfflineMetric m : metrics) {
			Object res = m.computeFromLogs(logDir);
			results.put(m, res);
		}
		return results;
	}
}