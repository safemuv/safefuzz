package ciexperiment.systematic;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import atlasdsl.Mission;
import exptrunner.jmetal.FuzzingSelectionsSolution;
import exptrunner.jmetal.InvalidMetrics;
import exptrunner.metrics.MetricsProcessing;
import faultgen.InvalidFaultFormat;
import fuzzingengine.FuzzingSelectionRecord;

public class RepeatSingleExpt extends ExptParams {
	private int runCountLimit;
	private int runCount = 0;
	private List<FuzzingSelectionRecord> fixedFaultInstances;
	private MetricsProcessing metricsProcessing;
	private Mission mission;
	private Mission exptMission;
	
	private FileWriter resFile;
	private String resFileName = "repeatedLog.res";
	
	private void setupResFile() {
		try {
			this.resFile = new FileWriter(resFileName);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public RepeatSingleExpt(MetricsProcessing mp, double runtime, int runCountLimit, Mission mission, String resFaultFile) throws FileNotFoundException, InvalidFaultFormat {
		super(runtime);
		this.runCountLimit = runCountLimit;
		this.runCount = 0;
		this.metricsProcessing = mp;
		this.mission = mission;
		this.exptMission = mission;
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
		// Ensure the necessary content is stored to track e.g. the configuration cost
		
		// TODO: this needs to call GenerateModelsExecutor in order to generate a fresh model from the CIExpt configuration 
		// Then load it to the exptMission object
	}

	public void logResults(String string) {
		// The FuzzingSelectionsSolution parameters
		FuzzingSelectionsSolution s = new FuzzingSelectionsSolution(mission, "", true, 0.0);
		if (fixedFaultInstances != null) {
			s.setAllContents(fixedFaultInstances);
		}
		try {
			metricsProcessing.readLogFiles(string, s);
			for (int i = 0; i < s.getNumberOfObjectives(); i++) {
				double m = s.getObjective(i);
				resFile.write(m + ",");
				System.out.println(metricsProcessing.getMetricByID(i) + "=" + m);
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

	public Mission getCurrentMissionObject() {
		return exptMission;
	}	
}
