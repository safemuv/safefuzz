package ciexperiment.systematic;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import exptrunner.jmetal.InvalidMetrics;
import exptrunner.metrics.Metrics;
import exptrunner.metrics.MetricsProcessing;

public class RunSameModel extends ExptParams {

	private int countLimit = 0;
	private int countCompleted = 0;
	private String modelFilePath;
	
	private MetricsProcessing metricsProcessing;
	
	private FileWriter resFile;
	private String resFileName = "repeatedLog.res";

	
	public RunSameModel(MetricsProcessing mp, double runtime, String modelFilePath, String resFile, int countLimit) {
		super(runtime);
		this.countLimit = countLimit;
		this.modelFilePath = modelFilePath;
	}
	
	public boolean completed() {
		return (countCompleted > countLimit);
	}

	public void printState() {

	}

	public void advance() {
		countCompleted++;
	}

	@Override
	public void logResults(String logDir, String modelFile, String ciClass) {
		System.out.println("Writing results to result file: " + resFileName);
		try {
			Map<Metrics,Object> metricRes = metricsProcessing.readMetricsFromLogFiles(logDir);
			resFile.write(modelFile + "," + ciClass + ",");
			for (Map.Entry<Metrics,Object> entry : metricRes.entrySet()) {
				Metrics m = entry.getKey();
				Object val = entry.getValue();
				System.out.println(m + "=" + val);
				resFile.write(entry.getValue().toString() + ",");
			}
			
			resFile.write("\n");
			resFile.flush();
		} catch (InvalidMetrics e1) {
			e1.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	public Optional<String> getNextFileName() {
		return null;
	}

}
