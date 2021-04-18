package test.logparsing;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import exptrunner.jmetal.InvalidMetrics;
import exptrunner.metrics.MetricsProcessing;
import exptrunner.metrics.MetricsProcessing.MetricStateKeys;
import exptrunner.metrics.Metrics;

public class TestParseLogs {
	public static void main(String [] args) {
		List<Metrics> metricList = new ArrayList<Metrics>();
		
		metricList.add(Metrics.PURE_MISSED_DETECTIONS);

		try {
			FileWriter tempLog = new FileWriter("test-templog.res");
			// Load the number of valid and missed detections from the given model
			MetricsProcessing mp = new MetricsProcessing(metricList, tempLog);
			
			// TODO: where to pass the number of verifications for the objects?
			mp.setMetricState(MetricStateKeys.MISSION_END_TIME, 1200.0);
			
			Map<Metrics,Object> metrics = mp.readMetricsFromLogFiles("/home/atlas/atlas/atlas-middleware/test-logfiles/logs");
			tempLog.close();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InvalidMetrics e) {
			e.printStackTrace();
		}
	}
}
