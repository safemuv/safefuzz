package exptrunner.jmetal;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Random;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.problem.Problem;

import atlasdsl.Mission;
import atlassharedclasses.FaultInstance;
import exptrunner.metrics.*;
import exptrunner.runner.RunExperiment;
import fuzzingengine.FuzzingSelectionRecord;

public class SAFEMUVEvaluationProblem implements Problem<FuzzingSelectionsSolution> {

	private static final long serialVersionUID = 1L;
	private String fakeLogFileName = "/home/atlas/atlas/atlas-middleware/middleware-java/tempres/customLog.res";

	private int runCount = 0;
	private Random rng;
	private Mission mission;
	private boolean actuallyRun;
	private double exptRunTime;
	private String logFileDir;

	// This gives the weights for these different goals
	//private Map<GoalsToCount, Integer> goalsToCount = new HashMap<GoalsToCount, Integer>();
	private Algorithm<List<FaultInstance>> algorithm;
	private MetricsProcessing metricsProcessing;

	private FileWriter tempLog;
	private int variableFixedSize;
	private int constraintCount = 0;
	
	private int popSize;
	
	public SAFEMUVEvaluationProblem(int popSize, Random rng, Mission mission, boolean actuallyRun, double exptRunTime,
			String logFileDir,
			//Map<GoalsToCount, Integer> goalsToCount,
			List<Metrics> metrics) throws IOException {
		this.rng = rng;
		this.popSize = popSize;
		this.mission = mission;
		this.exptRunTime = exptRunTime;
		this.logFileDir = logFileDir;
		this.actuallyRun = actuallyRun;
		//this.goalsToCount = goalsToCount;
		this.variableFixedSize = mission.getFaultsAsList().size();
		String fileName = new SimpleDateFormat("yyyyMMddHHmm").format(new Date());
		this.tempLog = new FileWriter("tempLog-" + fileName + ".res");
		metricsProcessing = new MetricsProcessing(metrics, tempLog);
		System.out.println(metrics.toString());
		
	}

	public int getNumberOfVariables() {
		// TODO: this is fixed
		return variableFixedSize;
	}

	public int getNumberOfObjectives() {
		return metricsProcessing.getMetrics().size();
	}

	public int getNumberOfConstraints() {
		return constraintCount;
	}

	public String getName() {
		return "SAFEMUVEvaluationProblem";
	}

	public void performSAFEMUVExperiment(FuzzingSelectionsSolution solution) throws InvalidMetrics {
		String exptTag = "exptGA-" + (runCount++);
		try {
			// TODO: should we generate the CSV file first or pass in the solution to do it?
			RunExperiment.doExperiment(mission, exptTag, solution, actuallyRun, exptRunTime);
			//metricsProcessing.readLogFiles(logFileDir, solution);
		} catch (InterruptedException | IOException e) {
			e.printStackTrace();
		}
	}

	public void evaluate(FuzzingSelectionsSolution solution) {
		try {
			performSAFEMUVExperiment(solution);
		} catch (InvalidMetrics e) {
			e.printStackTrace();
		}
		//return solution;
	}

	private void setupInitialPopulation(FuzzingSelectionsSolution fiss) {
		System.out.println("Setting up initial population...");
		List<FuzzingSelectionRecord> allFaults = new ArrayList<FuzzingSelectionRecord>();
		Collections.shuffle(allFaults, rng);
		// TODO: setup the initial population; 
		// This should be set up to confirm to the model
		System.out.println("Initial chromosome = " + fiss.toString());
	}


	public FuzzingSelectionsSolution createSolution() {
		int objectivesCount = metricsProcessing.getMetrics().size();
		// TODO: fix constructors for the fuzzing selection solutions
		FuzzingSelectionsSolution sol = new FuzzingSelectionsSolution(mission, "TAGTEST", actuallyRun, exptRunTime);
		setupInitialPopulation(sol);
		return sol;
	}

	public void setAlgorithm(Algorithm<List<FaultInstance>> algorithm) {
		this.algorithm = algorithm;
	}
}
