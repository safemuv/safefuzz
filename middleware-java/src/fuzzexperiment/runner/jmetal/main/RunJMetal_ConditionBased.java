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

public class RunJMetal_ConditionBased extends AbstractAlgorithmRunner {
	public static void main(String[] args) throws JMetalException {
		DSLLoader dslloader = new GeneratedDSLLoader();
		Mission mission;
		try {
			mission = dslloader.loadMission();
			
			double timingProbMut = 0.666;
			double participantProbMut = 0.333;
			double paramProbMut = 0.333;
			
			int numIterations = 1000;
			int populationSize = 1000;
			int offspringSize = 20;
			
			String scenarioStr = "S001";
			
			ExperimentType etype = ExperimentType.CONDITION_BASED_FUZZING_BOTH;
			
			JMetalExpt jmetalExpt = new JMetalExpt(scenarioStr, populationSize, offspringSize, numIterations, timingProbMut, participantProbMut, paramProbMut, etype);
			jmetalExpt.setActuallyRun(true);
			jmetalExpt.jMetalRun("condbasedfuzzing", mission);
		} catch (DSLLoadFailed e) {
			System.out.println("DSL loading failed - configuration problems");
			e.printStackTrace();
		} catch (ExptError e) {
			e.printStackTrace();
		}
	}
}
