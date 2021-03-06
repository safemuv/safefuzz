package fuzzexperiment.runner;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Properties;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.jmetal.FuzzingSelectionsSolution;
import fuzzexperiment.runner.metrics.*;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.support.FuzzingEngineSupport;

public class FuzzExptRunner {
	private boolean actuallyRun = true;
	private double timeLimit;
	private MetricHandler mh;
	private ExptParams eparams;
	private Mission baseMission;
	private String logPath;
	private StartFuzzingProcesses runner;
	
	private boolean startLaunchers;
	
	private int scenarioGenerationNumber = 0;
	private String scenarioID;
	
	private void readProperties() {
		Properties prop = new Properties();
		String fileName = "fuzzingexpt.config";
		InputStream is = null;
		try {
		    is = new FileInputStream(fileName);
		} catch (FileNotFoundException ex) {
			ex.printStackTrace();
		}
		try {
		    prop.load(is);
		    logPath = prop.getProperty("paths.ros.log");
		    String bashPath = prop.getProperty("paths.bash_script");
		    String workingPath = prop.getProperty("paths.working");
		    String middlewarePath = prop.getProperty("paths.middleware");
		    runner = new StartFuzzingProcesses(bashPath, workingPath, middlewarePath);
		    
		    is.close();
		} catch (IOException ex) {
			ex.printStackTrace();
		}
	}
	
	private void setup() throws DSLLoadFailed, IOException {
		DSLLoader loader = new GeneratedDSLLoader();
		baseMission = loader.loadMission();
		timeLimit = baseMission.getEndTime();
		FileWriter tempLog = new FileWriter("fuzzexpt-templog.log");
		readProperties();
	}
	
	public FuzzExptRunner(ExptParams ep, MetricHandler mh, boolean startLaunchers, String scenarioID) throws DSLLoadFailed, IOException {
		this.eparams = ep;
		this.mh = mh;
		this.startLaunchers = startLaunchers;
		this.scenarioID = scenarioID;
		setup();
	}
	
	public void useFakeRun() {
		actuallyRun = false;
	}
	
	public void writeToFile(Map<Metric,Object> results, String metricResFile) {
		// 
	}
	
	public void run() throws InterruptedException, IOException {
		// The core logic for the loop
		
		// Read fuzzing specification?
		// Read the experiment specification as a model - or use the generated CSV creator...
		// In the loop...
		
		while (!eparams.completed()) {
			Map<Metric, Double> metricResults;
			
			eparams.printState();
			// Get the CSV file consisting of a fuzzing experiment... internally either generate it 
			// in response to the metrics, or it comes from a constant list
			Optional<String> nextFile_o = eparams.getNextFuzzingCSVFileName();
			if (nextFile_o.isPresent()) {
				String csvFile = nextFile_o.get();
				String exptTag = csvFile;
				System.out.println("====================================================================================================");
				System.out.println("Running fuzzing experiments for CSV file name " + csvFile);
				List<FuzzingKeySelectionRecord> fuzzrecs = FuzzingEngineSupport.loadFuzzingRecords(baseMission, csvFile);
				System.out.println("Fuzzing records = " + fuzzrecs);
				FuzzingSelectionsSolution sol = new FuzzingSelectionsSolution(baseMission, exptTag, actuallyRun, timeLimit, fuzzrecs, eparams.getRunNum());
				
				if (actuallyRun) {
					int runNum = eparams.getRunNum();
					boolean regenerateScenarios = false;
					System.out.println("csvfile = " + csvFile);
					runner.runExptProcesses(scenarioID, baseMission, csvFile, timeLimit, sol, regenerateScenarios, startLaunchers, runNum);
				}
				
				// Assess the metrics (which the user defined using filled-in templates)
				// This should be done by the metrics handler now
				try {
					// Need to get the fuzzing selection records
					System.out.println("Fuzzing records = " + fuzzrecs);
					metricResults = mh.computeAllOffline(fuzzrecs, logPath);
					mh.printMetricsToOutputFile(csvFile, metricResults);
					eparams.advance(metricResults);
					
					
				} catch (MetricComputeFailure e) {
					// If we get metric computation failure, ignore the whole line
					// TODO: log the failure
					e.printStackTrace();
					eparams.advance();
				}
				
				System.out.println("----------------------------------------------------------------------------------------------------");
				eparams.printStateAfter();
			}	
		}
		System.out.println("Run completed");
		eparams.printFinal(mh.getMetrics());
	}
}
