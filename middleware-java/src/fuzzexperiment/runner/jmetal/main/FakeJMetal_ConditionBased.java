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

public class FakeJMetal_ConditionBased extends AbstractAlgorithmRunner {
	public static void main(String[] args) throws JMetalException {
		DSLLoader dslloader = new GeneratedDSLLoader();
		Mission mission;
		try {
			mission = dslloader.loadMission();
			double timingProbMut = 1.0;
			double participantProbMut = 0.0;
			double paramProbMut = 0.0;
			
			int numIterations = 18;
			int populationSize = 6;
			int offspringSize = 6;
			
			String scenarioStr = "S001";
			
			ExperimentType etype = ExperimentType.CONDITION_BASED_FUZZING_BOTH;
			
			JMetalExpt jmetalExpt = new JMetalExpt(scenarioStr, populationSize, offspringSize, numIterations, timingProbMut, participantProbMut, paramProbMut, etype);
			jmetalExpt.setActuallyRun(false);
			jmetalExpt.jMetalRun("condbasedfuzzing", mission);
		} catch (DSLLoadFailed e) {
			System.out.println("DSL loading failed - configuration problems");
			e.printStackTrace();
		} catch (ExptError e) {
			e.printStackTrace();
		}
	}
}
