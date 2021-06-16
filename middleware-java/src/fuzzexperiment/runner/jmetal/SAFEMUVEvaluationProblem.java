package fuzzexperiment.runner.jmetal;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Properties;
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
import fuzzexperiment.runner.metrics.FakeMetricHandler;
import fuzzexperiment.runner.metrics.Metric;
import fuzzexperiment.runner.metrics.MetricComputeFailure;
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
	private String bashPath;
	private String workingPath;
	private String middlewarePath;

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
			// logPath = prop.getProperty("paths.ros.log");
			bashPath = prop.getProperty("paths.bash_script");
			workingPath = prop.getProperty("paths.working");
			middlewarePath = prop.getProperty("paths.middleware");
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
		initialGenerator = new FuzzingExperimentGenerator(baseMission);
		runner = new StartFuzzingProcesses(bashPath, workingPath, middlewarePath);
	}

	public SAFEMUVEvaluationProblem(int popSize, Random rng, Mission mission, boolean actuallyRun, double exptRunTime,
			String logFileDir, List<OfflineMetric> metrics) throws IOException, DSLLoadFailed {
		this.rng = rng;
		// this.popSize = popSize;
		this.baseMission = mission;
		this.exptRunTime = exptRunTime;
		this.actuallyRun = actuallyRun;

		this.variableFixedSize = mission.getFaultsAsList().size();
		String resFileName = new SimpleDateFormat("yyyyMMddHHmm").format(new Date());
		this.tempLog = new FileWriter("tempLog-" + resFileName + ".res");

		if (actuallyRun) {
			metricHandler = new MetricHandler(metrics, resFileName);
		} else {
			metricHandler = new FakeMetricHandler(metrics, resFileName);

		}
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
			if (actuallyRun) {
				runner.codeGenerationROSFuzzing(baseMission, csvFileName);

				// Invoke the middleware (with the correct mission model and fuzzing spec!)
				// Invoke the CARS / call ROS launch scripts
				runner.doExperimentFromFile(exptTag, actuallyRun, timeLimit, csvFileName);
			}

			// Compute the metrics
			Map<Metric, Double> res = metricHandler.computeAllOffline(csvFileName);
			System.out.println("res = " + res);

			for (Map.Entry<Metric,Double> e : res.entrySet()) {
				Optional<Integer> jmetalNum_o = metricHandler.getMetricNumberInList(e.getKey());
				Double mval = e.getValue();
				if (jmetalNum_o.isPresent()) {
					int i = jmetalNum_o.get();
					solution.setObjective(i, mval);
				}
			}
		} catch (InterruptedException | IOException e) {
			e.printStackTrace();
		} catch (MetricComputeFailure e) {
			e.printStackTrace();
		} catch (NumberFormatException e) {
			System.out.println("Probably a metric returned something non-numeric");
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

	public FuzzingSelectionsSolution createSolution() {
		int objectivesCount = metricHandler.getMetrics().size();
		List<FuzzingSelectionRecord> recs = initialGenerator.generateExperiment(Optional.empty());
		System.out.println("createSolution - recs=" + recs);
		FuzzingSelectionsSolution sol = new FuzzingSelectionsSolution(baseMission, "TAGTEST", actuallyRun, exptRunTime, recs);
		System.out.println("Initial chromosome = " + sol.toString());
		return sol;
	}
}