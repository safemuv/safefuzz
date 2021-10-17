package fuzzexperiment.runner.jmetal.main;

import org.uma.jmetal.util.AbstractAlgorithmRunner;
import org.uma.jmetal.util.JMetalException;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.jmetal.ExptError;
import fuzzexperiment.runner.jmetal.JMetalExpt;
import fuzzexperiment.runner.jmetal.JMetalExpt.ExperimentType;

public class RunJMetal_timebased extends AbstractAlgorithmRunner {
	public static void main(String[] args) throws JMetalException {
		DSLLoader dslloader = new GeneratedDSLLoader();
		Mission mission;
		try {
			mission = dslloader.loadMission();
			
			double timingProbMut = 1/3;
			double participantProbMut = 1/3;
			double paramProbMut = 1/3;
			
			int numIterations = 50;
			
			ExperimentType etype = ExperimentType.FIXED_TIME_FUZZING;
			
			JMetalExpt jmetalExpt = new JMetalExpt(numIterations, timingProbMut, participantProbMut, paramProbMut, etype);
			jmetalExpt.setActuallyRun(true);
			jmetalExpt.jMetalRun("timebasedfuzzing", mission);
		} catch (DSLLoadFailed e) {
			System.out.println("DSL loading failed - configuration problems");
			e.printStackTrace();
		} catch (ExptError e) {
			e.printStackTrace();
		}
	}
}
