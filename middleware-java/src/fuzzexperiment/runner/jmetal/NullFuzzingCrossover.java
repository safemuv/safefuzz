package fuzzexperiment.runner.jmetal;

//Breaks up variable-length chromosomes X and Y and splits it into two new ones
//Takes the ones before the cut point - X_left, Y_left  
//Takes the ones after the cut point - X_right, Y_right

//Produces two new ones X_left, Y_right
//and Y_left, X_right
import org.uma.jmetal.operator.crossover.CrossoverOperator;
import org.uma.jmetal.solution.integersolution.IntegerSolution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;
import org.uma.jmetal.util.pseudorandom.RandomGenerator;

import atlassharedclasses.FaultInstance;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class NullFuzzingCrossover implements CrossoverOperator<FuzzingSelectionsSolution> {

	private double crossoverProbability;
	private Random randomGenerator;
	private static final long serialVersionUID = 1L;

	public NullFuzzingCrossover(double crossoverProbability, Random randomGenerator) {
		if (crossoverProbability < 0) {
			throw new JMetalException("Crossover probability is negative: " + crossoverProbability);
		}

		this.crossoverProbability = crossoverProbability;
		this.randomGenerator = randomGenerator;
	}

	public List<FuzzingSelectionsSolution> doCrossover(FuzzingSelectionsSolution cx, FuzzingSelectionsSolution cy) {
		List<FuzzingSelectionsSolution> output = new ArrayList<FuzzingSelectionsSolution>();
		return output;
	}

	public List<FuzzingSelectionsSolution> execute(List<FuzzingSelectionsSolution> solutions) {
		if (null == solutions) {
			throw new JMetalException("Null parameter");
		} else if (solutions.size() != 2) {
			throw new JMetalException("There must be two parents instead of " + solutions.size());
		}
		return doCrossover(solutions.get(0), solutions.get(1));
	}

	public double getCrossoverProbability() {
		return crossoverProbability;
	}

	public int getNumberOfRequiredParents() {
		return 2;
	}

	public int getNumberOfGeneratedChildren() {
		return 2;
	}

}
