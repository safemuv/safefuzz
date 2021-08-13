package fuzzingengine;

import java.util.List;

import fuzzingengine.conditionelements.FuzzingConditionElement;
import it.units.malelab.jgea.representation.tree.Tree;

public class FuzzingCondition {

	public FuzzingCondition(List<Tree<FuzzingConditionElement>> specTree) {
		// TODO: generate from this tree
	}

	public boolean isActive() {
		// TODO: implement tree parsing and condition checking
		return false;
	}

	public FuzzingCondition dup() {
		return null;
	}
}
