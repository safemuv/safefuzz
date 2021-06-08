package ciexperiment.systematic;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Optional;
import exptrunner.metrics.MetricsProcessing;

public class RunSameModel extends ExptResultsLogged {

	private int countLimit = 0;
	private int countCompleted = 0;
	private String modelFilePath;
	
	public RunSameModel(MetricsProcessing mp, double runtime, String modelFilePath, String resFileName, int countLimit) throws IOException {
		super(runtime);
		this.countLimit = countLimit;
		this.modelFilePath = modelFilePath;
		this.metricsProcessing = mp;
		this.resFile = new FileWriter(resFileName);
	}
	
	public boolean completed() {
		return (countCompleted >= countLimit);
	}

	public void printState() {

	}

	public void advance() {
		countCompleted++;
	}

	public Optional<String> getNextFileName() {
		return null;
	}

	public String getModelFile() {
		return modelFilePath;
	}

}
