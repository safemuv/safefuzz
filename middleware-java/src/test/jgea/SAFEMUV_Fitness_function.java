package test.jgea;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Function;

import it.units.malelab.jgea.core.fitness.BooleanFunctionFitness;
import it.units.malelab.jgea.core.fitness.CaseBasedFitness;
import it.units.malelab.jgea.core.fitness.BooleanFunctionFitness.TargetFunction;
import it.units.malelab.jgea.problem.booleanfunction.BooleanUtils;
import it.units.malelab.jgea.problem.booleanfunction.element.Element;
import it.units.malelab.jgea.representation.tree.Tree;

public class SAFEMUV_Fitness_function extends CaseBasedFitness<List<Tree<Element>>, boolean[], Boolean, Double> {

	public interface TargetFunction extends Function<boolean[], boolean[]> {
		String[] varNames();

		static TargetFunction from(final Function<boolean[], boolean[]> function, final String... varNames) {
			return new TargetFunction() {
				@Override
				public String[] varNames() {
					return varNames;
				}

				@Override
				public boolean[] apply(boolean[] values) {
					return function.apply(values);
				}
			};
		}
	}

	private static class ErrorRate implements Function<List<Boolean>, Double> {

		@Override
		public Double apply(List<Boolean> vs) {
			double errors = 0;
			for (Boolean v : vs) {
				errors = errors + (v ? 0d : 1d);
			}
			return errors / (double) vs.size();
		}

	}

	private static class Error implements BiFunction<List<Tree<Element>>, boolean[], Boolean> {

		
		public Error() {
			
		}

		public Boolean apply(List<Tree<Element>> solution, boolean[] observation) {
		// Fitness computation seems to be here
//			Map<String, Boolean> varValues = new LinkedHashMap<>();
//			for (int i = 0; i < targetFunction.varNames().length; i++) {
//				varValues.put(targetFunction.varNames()[i], observation[i]);
//			}
			return false;
		}
	}

	public SAFEMUV_Fitness_function(List<boolean[]> observations) {
			super(observations, new Error(), new ErrorRate());
		}

}
