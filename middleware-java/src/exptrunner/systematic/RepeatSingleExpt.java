package exptrunner.systematic;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Map;

import atlasdsl.Mission;
import exptrunner.jmetal.FuzzingSelectionsSolution;
import exptrunner.jmetal.InvalidMetrics;
import exptrunner.metrics.Metrics;
import exptrunner.metrics.MetricsProcessing;
import faultgen.InvalidFaultFormat;
import fuzzingengine.FuzzingSelectionRecord;

public class RepeatSingleExpt extends ExptParams {
	private int runCountLimit;
	private int runCount = 0;
	private List<FuzzingSelectionRecord> fixedFaultInstances;
	private MetricsProcessing metricsProcessing;
	private Mission mission;
	
	private FileWriter resFile;
	private String resFileName = "repeatedLog.res";
	private String faultFileName;
	
	private void setupResFile() {
		try {
			this.resFile = new FileWriter(resFileName);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public RepeatSingleExpt(MetricsProcessing mp, double runtime, int runCountLimit, Mission mission, String faultFileName, String resFaultFile) throws FileNotFoundException, InvalidFaultFormat {
		super(runtime);
		this.runCountLimit = runCountLimit;
		this.runCount = 0;
		this.metricsProcessing = mp;
		this.mission = mission;
		//this.fixedFaultInstances = FuzzingIO.loadFaultsFromCSV(faultFileName);
		this.faultFileName = faultFileName;
		this.resFileName = resFaultFile;
		setupResFile();
	}

	public boolean completed() {
		return (runCount >= runCountLimit);
	}

	public void printState() {
		System.out.println("runCount = " + runCount);		
	}

	public void advance() {
		runCount++;
	}

	public void logResults(String logDir) {
		System.out.println("Writing results to result file: " + resFileName);
		try {
			Map<Metrics,Object> metricRes = metricsProcessing.readMetricsFromLogFiles(logDir);
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

	public List<FuzzingSelectionRecord> specificFaults() {
		return fixedFaultInstances;
	}	
}
