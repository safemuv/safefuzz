package fuzzexperiment.runner.jmetal.main;

import org.uma.jmetal.util.AbstractAlgorithmRunner;
import org.uma.jmetal.util.JMetalException;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.jmetal.ExptError;
import fuzzexperiment.runner.jmetal.JMetalExpt;
import fuzzexperiment.runner.jmetal.SAFEMUVEvaluationProblem.ExperimentType;
import fuzzexperiment.runner.metrics.fake.*;

public class FakeJMetal_timebased extends AbstractAlgorithmRunner {
	public static void main(String[] args) throws JMetalException {
		DSLLoader dslloader = new GeneratedDSLLoader();
		Mission mission;
		try {
			mission = dslloader.loadMission();
			
			double timingProbMut = 0.666;
			double participantProbMut = 0.0;
			double paramProbMut = 0.0;

			int numIterations = 1000;
			int popSize = 10;
			int offspringSize = 10;
			
			String scenarioStr = "S001";
			
			// Do fake experiment to find a specific time
			ExperimentType etype = ExperimentType.FIXED_TIME_FUZZING;
			JMetalExpt jmetalExpt = new JMetalExpt(scenarioStr, popSize, offspringSize, numIterations, timingProbMut, participantProbMut, paramProbMut, etype);
			jmetalExpt.addSpecialMetric(new FindSpecificTime(100.0, 200.0));
			jmetalExpt.addSpecialMetric(new FindSpecificTime2(150.0, 190.0));
			
			jmetalExpt.setActuallyRun(false);
			jmetalExpt.jMetalRun("timebasedfake", mission);
		} catch (DSLLoadFailed e) {
			System.out.println("DSL loading failed - configuration problems");
			e.printStackTrace();
		} catch (ExptError e) {
			e.printStackTrace();
		}
	}
}
