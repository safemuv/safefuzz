package ciexperiment.systematic;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import exptrunner.jmetal.FuzzingSelectionsSolution;
import exptrunner.jmetal.InvalidMetrics;
import exptrunner.metrics.MetricsProcessing;
import faultgen.InvalidFaultFormat;

public class RunOnSetOfModels extends ExptParams {
	private List<String> modelFilePaths;
	private MetricsProcessing metricsProcessing;
	
	private FileWriter resFile;
	private String resFileName = "repeatedLog.res";
	
	private void setupResFile() {
		try {
			this.resFile = new FileWriter(resFileName);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public RunOnSetOfModels(MetricsProcessing mp, double runtime, List<String> modelFilePaths, String resFaultFile) throws FileNotFoundException, InvalidFaultFormat {
		super(runtime);
		this.modelFilePaths = modelFilePaths;
		this.metricsProcessing = mp;
		this.resFileName = resFaultFile;
		setupResFile();
	}

	public boolean completed() {
		return (modelFilePaths.size() == 0);
	}

	public void printState() {
		System.out.println("Pending models to process: " + modelFilePaths.size());	
	}

	public void advance() {
		modelFilePaths.remove(0);
	}


	public void logResults(String string) {
		// The FuzzingSelectionsSolution parameters
		FuzzingSelectionsSolution s = new FuzzingSelectionsSolution(null, "", true, 0.0);
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

	public Optional<String> getNextFileName() {
		return null;
	}
}
