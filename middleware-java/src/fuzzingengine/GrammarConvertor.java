package fuzzingengine;

import fuzzingengine.conditionelements.*;
import fuzzingengine.conditionelements.FuzzingConditionComparison.ComparisonOperation;
import it.units.malelab.jgea.representation.tree.Tree;

public class GrammarConvertor {
	public GrammarConvertor() {
		
	}
	
	public FuzzingConditionElement convert(Tree<String> stringTree) throws UnrecognisedComparison, UnrecognisedTreeNode {
		System.out.print("convert - Tree...\n");
		stringTree.prettyPrint(System.out);
		System.out.println();
		
		String s = stringTree.content();
		if (s.equals("<cond>")) {
			System.out.println("COND");
			System.out.println("Child 0 = " + stringTree.child(0));
			System.out.println("Child 1 = " + stringTree.child(1));
			System.out.println("Child 2 = " + stringTree.child(2));
			Tree<String> lhs = stringTree.child(0);
			String bincomp = stringTree.child(1).child(0).content();
			Tree<String> rhs = stringTree.child(2);
			
			FuzzingConditionElement compare = new FuzzingConditionComparison(convert(lhs), convert(rhs), convertCompare(bincomp));
			FuzzingConditionElementCond cond = new FuzzingConditionElementCond();
			cond.addSubcondition(compare);
			return cond;
		}
		
		if (s.equals("<var>")) {
			System.out.println("VAR");
			String varname = stringTree.child(0).content();
			return new FuzzingConditionVariable(varname);
		}
		
		if (s.equals("<expr>")) {
			return convert(stringTree.child(0));
		}
		
		if (s.equals("<int>")) {
			String valStr = stringTree.child(0).content();
			int v = Integer.valueOf(valStr);
			return new FuzzingConditionConstant(v);
		}
		
		throw new UnrecognisedTreeNode(s);
		
	}
	
	private ComparisonOperation convertCompare(String compStr) throws UnrecognisedComparison {
		if (compStr.equals(".EQUALS")) {
			return ComparisonOperation.EQUALS;
		}
		
		if (compStr.equals(".NOT_EQUALS")) {
			return ComparisonOperation.NOT_EQUALS;
		}
		
		if (compStr.equals(".LESS_THAN")) {
			return ComparisonOperation.LESS_THAN;
		}
		
		if (compStr.equals(".GREATER_THAN")) {
			return ComparisonOperation.GREATER_THAN;
		}
		
		throw new UnrecognisedComparison(compStr);
	}
}
