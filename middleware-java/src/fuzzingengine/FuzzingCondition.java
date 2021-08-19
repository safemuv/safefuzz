package fuzzingengine;

import fuzzingengine.conditionelements.FuzzingConditionElement;
import it.units.malelab.jgea.representation.tree.Tree;

public class FuzzingCondition {
	private Tree<FuzzingConditionElement> specTree;
	
	public FuzzingCondition(Tree<FuzzingConditionElement> specTree) {
		this.specTree = specTree; 
	}

	public boolean isActive() {
		// TODO: implement tree parsing and condition checking
		return false;
	}
	
	public <T> Tree<T> copyTree(Tree<T> inT) {
		// TODO: implement actual copy here
		System.out.println("TODO - NEED TO PROPERLY COPY THE TREE HERE");
		return inT;
	}

	public FuzzingCondition dup() {
		return new FuzzingCondition(copyTree(specTree));
	}

	public static FuzzingCondition parseString(String startSpec) {
		Tree<FuzzingConditionElement> t = new Tree<FuzzingConditionElement>(null, null);
		return new FuzzingCondition(t);
	}
}
