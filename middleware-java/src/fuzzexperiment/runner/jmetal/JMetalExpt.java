package fuzzexperiment.runner.jmetal;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.Properties;
import java.util.Random;
import java.util.stream.Collectors;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.operator.mutation.MutationOperator;
import org.uma.jmetal.operator.selection.SelectionOperator;
import org.uma.jmetal.operator.selection.impl.TournamentSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.util.AbstractAlgorithmRunner;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.SequentialSolutionListEvaluator;


import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import fuzzexperiment.runner.jmetal.SAFEMUVEvaluationProblem.ExperimentType;
import fuzzexperiment.runner.jmetal.customalg.NSGAII_JRH;
import fuzzexperiment.runner.jmetal.grammar.Grammar;
import fuzzexperiment.runner.metrics.Metric;
import fuzzexperiment.runner.metrics.OfflineMetric;
import fuzzexperiment.runner.rmkg.RMKGInterface;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.spec.GeneratedFuzzingSpec;

public class JMetalExpt extends AbstractAlgorithmRunner {

	private String GRAMMAR_FILE = System.getProperty("user.home") + "/academic/atlas/atlas-middleware/grammar/safemuv-fuzzing-cond.bnf";

	private final boolean USE_CROSSOVER = true;

	private int populationSize;
	private int offspringPopulationSize;
	
	private String crossoverLogFile = "crossover.log";
	
	static private boolean actuallyRun = false;
	static private double exptRunTime = 600.0;

	private int maxIterations = 50;

	static double crossoverProb = 0.5;
	static double mutationProb = 0.8;

	static private String referenceParetoFront = "";

	private double timingMutProb;
	private double participantsMutProb;
	private double paramMutProb;

	private List<OfflineMetric> specialMetrics = new ArrayList<OfflineMetric>();

	private ExperimentType etype;

	private String logPath;

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
			is.close();
		} catch (IOException ex) {
			ex.printStackTrace();
		}
	}

	public JMetalExpt(int popSize, int offspringPopSize, int maxIterations, double timingMutProb, double participantsMutProb, double paramMutProb,
			ExperimentType etype) {
		this.populationSize = popSize;
		this.offspringPopulationSize = offspringPopSize;
		this.maxIterations = maxIterations;
		this.timingMutProb = timingMutProb;
		this.participantsMutProb = participantsMutProb;
		this.paramMutProb = paramMutProb;
		this.etype = etype;
		readProperties();
	}
	
	public void addSpecialMetric(OfflineMetric om) {
		specialMetrics.add(om);
	}

	public void jMetalRun(String tag, Mission mission) throws ExptError, DSLLoadFailed {

		List<Metric> allMetrics = new ArrayList<Metric>(mission.getAllMetrics());
		String tagDated = tag + (new SimpleDateFormat("yyyy-MM-dd").format(new Date()));

		List<OfflineMetric> metrics;

		// Only include the mission metrics if we're actually running it!
		// otherwise there are no log files to process
		if (actuallyRun) {
			metrics = allMetrics.stream()
					.filter(e -> (e instanceof OfflineMetric))
					.map(e -> (OfflineMetric) e)
					.collect(Collectors.toList());
		} else {
			metrics = new ArrayList<OfflineMetric>();
		}

		for (OfflineMetric m : specialMetrics) {
			metrics.add(m);
		}

		Random problemRNG = new Random();
		Random crossoverRNG = new Random();
		Random mutationRNG = new Random();

		Problem<FuzzingSelectionsSolution> problem;

		try {
			FuzzingEngine fuzzEngine = GeneratedFuzzingSpec.createFuzzingEngine(mission, false);
			Grammar<String> g = Grammar.fromFile(new File(GRAMMAR_FILE));
						
			problem = new SAFEMUVEvaluationProblem(g, populationSize, problemRNG, mission, actuallyRun, exptRunTime,
					logPath, metrics, etype, tagDated, RMKGInterface.REGENERATE_SCENARIOS);

			Algorithm<List<FuzzingSelectionsSolution>> algorithm;
			
			FuzzingCrossoverOperation crossover;
			
			MutationOperator<FuzzingSelectionsSolution> mutation;
			SelectionOperator<List<FuzzingSelectionsSolution>, FuzzingSelectionsSolution> selection;
			SolutionListEvaluator<FuzzingSelectionsSolution> evaluator;
			Comparator<FuzzingSelectionsSolution> dominanceComparator;

			if (USE_CROSSOVER) {
				crossover = new FuzzingCrossoverMergeKeys(crossoverProb, crossoverRNG, crossoverLogFile);
			} else {
				crossover = new NullFuzzingCrossover(crossoverProb, crossoverRNG, crossoverLogFile);
			}

			mutation = new FuzzingSelectionsMutation(g, mutationRNG, mission, fuzzEngine, "mutation.log", timingMutProb,
					paramMutProb, participantsMutProb);

			selection = new TournamentSelection<FuzzingSelectionsSolution>(5);
			dominanceComparator = new DominanceComparator<>();
			evaluator = new SequentialSolutionListEvaluator<FuzzingSelectionsSolution>();

			int matingPoolSize = offspringPopulationSize;
			
			algorithm = new NSGAII_JRH(problem, maxIterations, populationSize, matingPoolSize,
					offspringPopulationSize, crossover, mutation, selection, dominanceComparator, evaluator);

			long startTime = System.currentTimeMillis();
			algorithm.run();
			List<FuzzingSelectionsSolution> population = algorithm.getResult();
			double duration = (System.currentTimeMillis() - startTime);
			System.out.println("Total execution time: " + duration + "ms, " + (duration / 1000) + " seconds");

			printFinalSolutionSet(population);
			((FuzzingSelectionsMutation)mutation).closeLog();
			((FuzzingCrossoverOperation)crossover).closeLog();
			
			((NSGAII_JRH)algorithm).logFinalSolutionsCustom("jmetal-finalPopNonDom.res", "jmetal-finalPop.res");
			
			
			if (!referenceParetoFront.equals("")) {
				printQualityIndicators(population, referenceParetoFront);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void setActuallyRun(boolean actuallyRun) {
		this.actuallyRun = actuallyRun;
	}
}
