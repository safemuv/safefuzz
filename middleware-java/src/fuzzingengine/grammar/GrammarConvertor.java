package fuzzingengine.grammar;

import fuzzingengine.UnrecognisedBinOp;
import fuzzingengine.UnrecognisedComparison;
import fuzzingengine.UnrecognisedTreeNode;
import fuzzingengine.UnrecognisedUnOp;
import fuzzingengine.grammar.conditionelements.*;
import fuzzingengine.grammar.conditionelements.FuzzingConditionComparison.ComparisonOperation;
import fuzzingengine.grammar.conditionelements.FuzzingConditionElementCondBinOp.FuzzingConditionBinLogicOp;
import fuzzingengine.grammar.conditionelements.FuzzingConditionElementCondUnOp.FuzzingConditionUnLogicOp;
import it.units.malelab.jgea.representation.tree.Tree;

public class GrammarConvertor {
	protected final boolean DEBUG_GRAMMER_CONVERT = true; 
	
	public GrammarConvertor() {
		
	}
	
	protected FuzzingConditionElement createVariableEntry(String varName) {
		return new FuzzingConditionVariable(varName);
	}
	
	public FuzzingConditionElement convert(Tree<String> stringTree) throws UnrecognisedComparison, UnrecognisedTreeNode, UnrecognisedUnOp, UnrecognisedBinOp {
		
		if (DEBUG_GRAMMER_CONVERT) {
			System.out.print("convert - Tree...\n");
			stringTree.prettyPrint(System.out);
			System.out.println();
		}
		
		String s = stringTree.content();
		// Binary combined condition
		if (s.equals("<compcond>") && stringTree.nChildren() == 3) {
			if (DEBUG_GRAMMER_CONVERT) {
				System.out.println("COMPCOND - BINARY");
				System.out.println("Child 0 = " + stringTree.child(0));
				System.out.println("Child 1 = " + stringTree.child(1));
				System.out.println("Child 2 = " + stringTree.child(2));
			}
			Tree<String> lhs = stringTree.child(0);
			String bincomp = stringTree.child(1).child(0).content();
			Tree<String> rhs = stringTree.child(2);
			
			FuzzingConditionElement condlhs = convert(lhs);
			FuzzingConditionBinLogicOp op = convertBinLogicOp(bincomp);
			FuzzingConditionElement condrhs = convert(rhs);
			
			FuzzingConditionElementCond cond = new FuzzingConditionElementCondBinOp(condlhs, condrhs, op);
			return cond;
		}
		
		// Unary combined condition
		if (s.equals("<compcond>") && stringTree.nChildren() == 2) {
			if (DEBUG_GRAMMER_CONVERT) {
				System.out.println("COMPCOND - UNARY");
				System.out.println("Child 0 = " + stringTree.child(0));
				System.out.println("Child 1 = " + stringTree.child(1));
			}
			String unop = stringTree.child(0).child(0).content();
			Tree<String> c = stringTree.child(1);
			
			FuzzingConditionElement inner = convert(c);
			FuzzingConditionUnLogicOp op = convertUnLogicOp(unop);
			
			FuzzingConditionElementCond cond = new FuzzingConditionElementCondUnOp(inner,op);
			return cond;
		}
		
		// Unary combined condition
		if (s.equals("<compcond>") && stringTree.nChildren() == 1) {
			if (DEBUG_GRAMMER_CONVERT) {
				System.out.println("COMPCOND - BASIC");
			}
			Tree<String> c = stringTree.child(0);
			return convert(c);
		}
		
		// Unary combined condition
		if (s.equals("<compcond>") && stringTree.nChildren() == 2) {
			if (DEBUG_GRAMMER_CONVERT) {
				System.out.println("COMPCOND");
				System.out.println("Child 0 = " + stringTree.child(0));
				System.out.println("Child 1 = " + stringTree.child(1));
				System.out.println("Child 2 = " + stringTree.child(2));
			}
			String unop = stringTree.child(0).child(0).content();
			Tree<String> c = stringTree.child(1);
			
			FuzzingConditionElement inner = convert(c);
			FuzzingConditionUnLogicOp op = convertUnLogicOp(unop);
			
			FuzzingConditionElementCond cond = new FuzzingConditionElementCondUnOp(inner,op);
			return cond;
		}
		
		if (s.equals("<basic_cond>")) {
			if (DEBUG_GRAMMER_CONVERT) {
				System.out.println("BASIC_COND");
				System.out.println("Child 0 = " + stringTree.child(0));
				System.out.println("Child 1 = " + stringTree.child(1));
				System.out.println("Child 2 = " + stringTree.child(2));
			}
			Tree<String> lhs = stringTree.child(0);
			String bincomp = stringTree.child(1).child(0).content();
			Tree<String> rhs = stringTree.child(2);
			
			FuzzingConditionElement compare = new FuzzingConditionComparison(convert(lhs), convert(rhs), convertCompare(bincomp));
			FuzzingConditionElementCond cond = new FuzzingConditionElementCond();
			cond.addSubcondition(compare);
			return cond;
		}
		
		if (s.equals("<var>")) {
			if (DEBUG_GRAMMER_CONVERT) {
				System.out.println("VAR");
			}
			String varname = stringTree.child(0).content();
			return createVariableEntry(varname);
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
	
	private FuzzingConditionUnLogicOp convertUnLogicOp(String compStr) throws UnrecognisedUnOp {
		if (compStr.equals(".NOT")) {
			return FuzzingConditionUnLogicOp.NOT;
		}
		
		throw new UnrecognisedUnOp(compStr);
	}

	private FuzzingConditionBinLogicOp convertBinLogicOp(String compStr) throws UnrecognisedBinOp {
		if (compStr.equals(".AND")) {
			return FuzzingConditionBinLogicOp.AND;
		}
		
		if (compStr.equals(".OR")) {
			return FuzzingConditionBinLogicOp.OR;
		}
		
		throw new UnrecognisedBinOp(compStr);
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
