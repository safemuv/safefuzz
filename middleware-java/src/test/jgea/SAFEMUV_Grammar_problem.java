package test.jgea;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Function;

import it.units.malelab.jgea.core.fitness.BooleanFunctionFitness;
import it.units.malelab.jgea.problem.booleanfunction.BooleanUtils;
import it.units.malelab.jgea.problem.booleanfunction.FormulaMapper;
import it.units.malelab.jgea.problem.booleanfunction.EvenParity.TargetFunction;
import it.units.malelab.jgea.problem.booleanfunction.element.Element;
import it.units.malelab.jgea.representation.grammar.Grammar;
import it.units.malelab.jgea.representation.grammar.GrammarBasedProblem;
import it.units.malelab.jgea.representation.tree.Tree;

public class SAFEMUV_Grammar_problem implements GrammarBasedProblem<String, List<Tree<Element>>, Double> {

	private static class TargetFunction implements BooleanFunctionFitness.TargetFunction {

		private final String[] varNames;

		public TargetFunction(int size) {
			varNames = new String[size];
			for (int i = 0; i < size; i++) {
				varNames[i] = "b" + i;
			}
		}

		@Override
		public boolean[] apply(boolean[] arguments) {
			int count = 0;
			for (boolean argument : arguments) {
				count = count + (argument ? 1 : 0);
			}
			return new boolean[] { (count % 2) == 1 };
		}

		@Override
		public String[] varNames() {
			return varNames;
		}
	}
	
	public SAFEMUV_Grammar_problem(final int size) throws IOException {
		grammar = Grammar.fromFile(new File("/home/jharbin/academic/atlas/atlas-middleware/grammar/safemuv-fuzzing-cond.bnf"));
		System.out.println("Grammar rules = " + grammar.getRules());
		System.out.println("Starting symbol = " + grammar.getStartingSymbol());
		
		solutionMapper = new FormulaMapper();
		//TargetFunction targetFunction = new TargetFunction(size);
	
		// So far this is a dummy fitness function that just returns false for everything
		fitnessFunction = new SAFEMUV_Fitness_function();
	}

	private final Grammar<String> grammar;
	private final Function<Tree<String>, List<Tree<Element>>> solutionMapper;
	private final Function<List<Tree<Element>>, Double> fitnessFunction;

	@Override
	public Grammar<String> getGrammar() {
		return grammar;
	}

	@Override
	public Function<Tree<String>, List<Tree<Element>>> getSolutionMapper() {
		return solutionMapper;
	}

	@Override
	public Function<List<Tree<Element>>, Double> getFitnessFunction() {
		return fitnessFunction;
	}

}
