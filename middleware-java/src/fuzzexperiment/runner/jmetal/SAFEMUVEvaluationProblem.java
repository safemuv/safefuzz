package fuzzexperiment.runner.jmetal;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.Scanner;
import java.util.stream.Collectors;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.Solution;

import atlasdsl.Mission;
import atlasdsl.faults.Fault;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import atlassharedclasses.FaultInstance;
import carsspecific.ros.codegen.ROSCodeGen;
import exptrunner.metrics.*;
import exptrunner.runner.RunExperiment;
import fuzzexperiment.runner.StartFuzzingProcesses;
import fuzzexperiment.runner.metrics.MetricHandler;
import fuzzexperiment.runner.metrics.OfflineMetric;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.exptgenerator.FuzzingExperimentGenerator;
import fuzzingengine.spec.GeneratedFuzzingSpec;
import middleware.atlascarsgenerator.ConversionFailed;
import exptrunner.*;
import utils.Pair;

public class SAFEMUVEvaluationProblem implements Problem<FuzzingSelectionsSolution> {

	private static final long serialVersionUID = 1L;
	private int runCount = 0;
	private Random rng;
	private Mission baseMission;
	private boolean actuallyRun;
	private double exptRunTime;
	private double timeLimit;

	private MetricHandler metricHandler;
	private StartFuzzingProcesses runner;

	private FileWriter tempLog;
	private int variableFixedSize;
	private int constraintCount = 0;
	
	private FuzzingExperimentGenerator initialGenerator;
	
	private void setup() throws DSLLoadFailed, IOException {
		DSLLoader loader = new GeneratedDSLLoader();
		baseMission = loader.loadMission();
		timeLimit = baseMission.getEndTime();
		FileWriter tempLog = new FileWriter("fuzzexpt-templog.log");
		//readProperties();
	}
	
	public SAFEMUVEvaluationProblem(int popSize, Random rng, Mission mission, boolean actuallyRun, double exptRunTime,
			String logFileDir, List<OfflineMetric> metrics) throws IOException, DSLLoadFailed {
		this.rng = rng;
		//this.popSize = popSize;
		this.baseMission = mission;
		this.exptRunTime = exptRunTime;
		this.actuallyRun = actuallyRun;

		this.variableFixedSize = mission.getFaultsAsList().size();
		String resFileName = new SimpleDateFormat("yyyyMMddHHmm").format(new Date());
		this.tempLog = new FileWriter("tempLog-" + resFileName + ".res");
		metricHandler = new MetricHandler(metrics, resFileName);
		System.out.println(metrics.toString());
		setup();		
	}

	public int getNumberOfVariables() {
		return variableFixedSize;
	}

	public int getNumberOfObjectives() {
		return metricHandler.getMetrics().size();
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
			
			String csvFileName = solution.getCSVFileName();
			// TODO: ensure a clear simulation run
						
			// Generate the ROS configuration files, e.g. modified launch scripts, YAML
			// config files etc for this CSV definition experimental run
			runner.codeGenerationROSFuzzing(baseMission, csvFileName);
			
			// Invoke the middleware (with the correct mission model and fuzzing spec!)
			// Invoke the CARS / call ROS launch scripts
			runner.doExperimentFromFile(exptTag, actuallyRun, timeLimit, csvFileName);
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
	}

	private void setupInitialPopulation(FuzzingSelectionsSolution fiss) {
		System.out.println("Setting up initial population...");
		List<FuzzingSelectionRecord> recs = initialGenerator.generateExperiment(null);
		for (int i = 0; i < recs.size(); i++) {
			fiss.setVariable(i, recs.get(i));
		}
		
		System.out.println("Initial chromosome = " + fiss.toString());
	}


	public FuzzingSelectionsSolution createSolution() {
		int objectivesCount = metricHandler.getMetrics().size();
		// TODO: fix constructors for the fuzzing selection solutions
		// TODO: use the existing experiment generator code for here!
		FuzzingSelectionsSolution sol = new FuzzingSelectionsSolution(baseMission, "TAGTEST", actuallyRun, exptRunTime);
		setupInitialPopulation(sol);
		return sol;
	}
}
