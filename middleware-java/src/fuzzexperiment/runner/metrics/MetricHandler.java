package fuzzexperiment.runner.metrics;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import atlasdsl.Mission;

public class MetricHandler {
	// TODO: how to supply content to the metrics? - e.g. structure of the solution?
	private List<OfflineMetric> metrics = new ArrayList<OfflineMetric>();
	private FileWriter resFile;
	private String resFileName;

	public void setupFileIfNotDone() throws IOException {
		if (resFile == null) {
			resFile = new FileWriter(resFileName);
		}
	}
	
	public MetricHandler(List<OfflineMetric> metrics, String resFileName) throws IOException {
		this.metrics = metrics;
		this.resFileName = resFileName;
		setupFileIfNotDone();
	}

	public MetricHandler(Mission mission, String resFileName) throws IOException {
		this.resFileName = resFileName;
		setupFileIfNotDone();
		Set<Metric> missionMetrics = mission.getAllMetrics();
		for (Metric m : missionMetrics) {
			if (m instanceof OfflineMetric) {
				metrics.add((OfflineMetric) m);
			}
		}
	}

	public void printHeader() throws IOException {
		for (OfflineMetric m : metrics) {
			resFile.write(m.getClass().getSimpleName());
		}
	}

	public void printMetrics(Map<Metric, Object> metricRes) throws IOException {
		System.out.println("Writing results to result file: " + resFileName);
		for (OfflineMetric m : metrics) {
			Object val = metricRes.get(m);
			System.out.println(m + "=" + val);
			resFile.write(val.toString() + ",");
		}
		resFile.write("\n");
		resFile.flush();
	}
	
	public void closeRes() throws IOException {
		resFile.close();
	}

	public Map<Metric, Object> computeAllOffline(String logDir) throws MetricComputeFailure {
		Map<Metric, Object> results = new HashMap<Metric, Object>();
		for (OfflineMetric m : metrics) {
			Object res = m.computeFromLogs(logDir);
			results.put(m, res);
		}
		return results;
	}
}