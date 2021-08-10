package test.jgea;

import java.util.List;
import java.util.function.Function;
import it.units.malelab.jgea.problem.booleanfunction.element.Element;
import it.units.malelab.jgea.representation.tree.Tree;

public class SAFEMUV_Fitness_function implements Function<List<Tree<Element>>, Double> {
	public Double apply(List<Tree<Element>> t) {
		Double score = 0.0;
		if (t.size() > 0) {
			Tree<Element> e = t.get(0);
			System.out.println("Tree elements = " + e);
			Tree<Element> e1 = e.child(0);
			Element eleft = e1.child(0).content();
			Element eright = e1.child(1).content();
			
			System.out.println("e1 = " + e1 + ",eleft = " + eleft + ",eright = " + eright);
			
//			if (eleft.equals("testvar2")) {
//				score += 1.0;
//			}
//			
//			//if (eright.equals()) {
////				score += 1.0;
//			//}
			
			return score;
			
		} else {
			return 0.0;
		}
	}
}
