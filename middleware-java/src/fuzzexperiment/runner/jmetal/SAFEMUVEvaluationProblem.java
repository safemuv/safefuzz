package fuzzexperiment.runner.jmetal;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Properties;
import java.util.Random;
import org.uma.jmetal.problem.Problem;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.StartFuzzingProcesses;
import fuzzexperiment.runner.metrics.FakeMetricHandler;
import fuzzexperiment.runner.metrics.Metric;
import fuzzexperiment.runner.metrics.MetricComputeFailure;
import fuzzexperiment.runner.metrics.MetricHandler;
import fuzzexperiment.runner.metrics.OfflineMetric;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.exptgenerator.FuzzingExperimentGenerator;
import fuzzingengine.exptgenerator.FuzzingExperimentModifier;
import fuzzingengine.exptgenerator.FuzzingTimeSpecificationGenerator;
import fuzzingengine.exptgenerator.FuzzingTimeSpecificationGeneratorDualCond;
import fuzzingengine.exptgenerator.FuzzingTimeSpecificationGeneratorStartEnd;
import fuzzingengine.support.FuzzingEngineSupport;
import fuzzexperiment.runner.jmetal.grammar.Grammar;
import fuzzexperiment.runner.jmetal.grammar.GrowGrammarTreeFactory;

public class SAFEMUVEvaluationProblem implements Problem<FuzzingSelectionsSolution> {

	private static final long serialVersionUID = 1L;

	private static final int MAX_TRIES_GENERATING_EXPERIMENT = 20;
	
	public enum ExperimentType {
		FIXED_TIME_FUZZING, 
		CONDITION_BASED_FUZZING_START, 
		CONDITION_BASED_FUZZING_BOTH
	}
	
	private Random rng;
	private Mission baseMission;
	private boolean actuallyRun;
	private double exptRunTime;
	private double timeLimit;
	private ExperimentType etype;

	private MetricHandler metricHandler;
	private StartFuzzingProcesses runner;

	private FileWriter tempLog;
	private int variableFixedSize;
	private int constraintCount = 0;
	private int runCount = 0;

	private FuzzingExperimentGenerator initialGenerator;
	private String bashPath;
	private String workingPath;
	private String middlewarePath;
	private String logPath;
	
	private int MIN_GRAMMAR_HEIGHT = 3;
	private int MAX_GRAMMAR_HEIGHT = 7;
	
	Grammar<String> grammar;

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
			logPath = workingPath + "/logs/";
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
		
		runner = new StartFuzzingProcesses(bashPath, workingPath, middlewarePath);

		FuzzingTimeSpecificationGenerator tgen = new FuzzingTimeSpecificationGeneratorStartEnd(baseMission, new Random());
		if (etype == ExperimentType.FIXED_TIME_FUZZING) {
			tgen = new FuzzingTimeSpecificationGeneratorStartEnd(baseMission, new Random());
		} 
		
		if (etype == ExperimentType.CONDITION_BASED_FUZZING_BOTH) {
			GrowGrammarTreeFactory<String> gen = new GrowGrammarTreeFactory(MAX_GRAMMAR_HEIGHT, grammar);
			tgen = new FuzzingTimeSpecificationGeneratorDualCond(baseMission, new Random(), gen);
		}
		
		initialGenerator = new FuzzingExperimentModifier(tgen, baseMission);
		
		System.out.println("initialGenerator class = " + initialGenerator.getClass().getSimpleName());
		
		System.out.println("SAFEMUVEvaluationProblem: Grammar rules = " + grammar.getRules());
		System.out.println("SAFEMUVEvaluationProblem: Starting symbol = " + grammar.getStartingSymbol());
		
	}

	public SAFEMUVEvaluationProblem(Grammar<String> g, int popSize, Random rng, Mission mission, boolean actuallyRun, double exptRunTime,
			String logFileDir, List<OfflineMetric> metrics, ExperimentType etype) throws IOException, DSLLoadFailed {
		this.rng = rng;
		this.baseMission = mission;
		this.exptRunTime = exptRunTime;
		this.actuallyRun = actuallyRun;
		this.grammar = g;
		this.etype = etype;

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
				runner.doExperimentFromFile(exptTag, actuallyRun, timeLimit, csvFileName, Optional.empty());
			}

			// Compute the metrics
			System.out.println("csvFileName = " + csvFileName);
			List<FuzzingKeySelectionRecord> fuzzrecs = FuzzingEngineSupport.loadFuzzingRecords(baseMission, csvFileName);
			Map<Metric, Double> res = metricHandler.computeAllOffline(fuzzrecs, logPath);
			System.out.println("res = " + res);
			
			tempLog.write(csvFileName + ",");

			for (Map.Entry<Metric,Double> e : res.entrySet()) {
				Optional<Integer> jmetalNum_o = metricHandler.getMetricNumberInList(e.getKey());
				Metric m = e.getKey();
				Double mval = e.getValue();
				System.out.println("Metric: " + m.getClass().getSimpleName() + "=" + mval);
				tempLog.write(m.getClass().getSimpleName() + "=" + mval + ",");
				if (jmetalNum_o.isPresent()) {
					int i = jmetalNum_o.get();
					
					if (m.optimiseDirection() == Metric.MetricDirection.HIGHEST) {
						// JMetal's comparison expects a lower value as better
						// so when the metric should be increased, invert the sign
						solution.setObjective(i, -mval);
					} else {
						solution.setObjective(i, mval);
					}
				}
			}
			tempLog.write("\n");
			tempLog.flush();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (MetricComputeFailure e) {
			e.printStackTrace();
		} catch (NumberFormatException e) {
			System.out.println("Probably a metric returned something non-numeric");
			e.printStackTrace();
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
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
		int tryCount = 0;
		
		boolean cont = true;
		List<FuzzingSelectionRecord> recs;
		
		recs = initialGenerator.generateExperiment(Optional.empty());
		if (recs.size() > 0) {
			cont = false;
		}
		while (cont) {
			recs = initialGenerator.generateExperiment(Optional.empty());
			// This is to prevent infinite loop if the models have no fuzzing probabalities defined
			// It will eventually give up generating if no records are produced
			if (tryCount++ > MAX_TRIES_GENERATING_EXPERIMENT) {
				cont = false;
			}
		}
		System.out.println("createSolution - recs=" + recs);
		FuzzingSelectionsSolution sol = new FuzzingSelectionsSolution(baseMission, "TAGTEST", actuallyRun, exptRunTime, recs);
		System.out.println("Initial chromosome = " + sol.toString());
		return sol;
	}
}
