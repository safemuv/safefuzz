package fuzzingengine.grammar;

import fuzzingengine.InvalidCondition;
import fuzzingengine.UnrecognisedBinOp;
import fuzzingengine.UnrecognisedComparison;
import fuzzingengine.UnrecognisedTreeNode;
import fuzzingengine.UnrecognisedUnOp;
import fuzzingengine.grammar.conditionelements.*;
import it.units.malelab.jgea.representation.tree.Tree;
import middleware.core.ATLASCore;

public class FuzzingCondition {
	private FuzzingConditionElement elementTree;
	private Tree<String> stringTree;
	private GrammarConvertor conv = new GrammarConvertor();

	public FuzzingCondition(Tree<String> stringTree) {
		this.stringTree = stringTree;
	}
	
	public FuzzingCondition(Tree<String> stringTree, GrammarConvertor conv) {
		this.stringTree = stringTree;
		this.conv = conv;
	}

	public void doConversion() throws UnrecognisedComparison, UnrecognisedTreeNode, UnrecognisedUnOp, UnrecognisedBinOp {
		this.elementTree = conv.convert(stringTree);
	}

	public Tree<String> getTree() {
		return stringTree;
	}

	public boolean evaluate(String vehicleName) {
		// Not sure why this isn't being called during fuzzing engine setup
		if (this.elementTree == null) {
			try {
				doConversion();
			} catch (UnrecognisedComparison e) {
				e.printStackTrace();
				return false;
			} catch (UnrecognisedTreeNode e) {
				e.printStackTrace();
				return false;
			} catch (UnrecognisedUnOp e) {
				e.printStackTrace();
			} catch (UnrecognisedBinOp e) {
				e.printStackTrace();
			}
		}
		
		ATLASCore core = ATLASCore.getCore();
		Object res = this.elementTree.evaluate(core, vehicleName);
		if (res instanceof Boolean) {
			Boolean resB = (Boolean) res;
			return resB;
		} else {
			System.out.println("Returning FALSE - non-boolean returned in evaluating condition " + this);
			return false;
		}
	}

	public <T> Tree<T> copyTree(Tree<T> inT) {
		return Tree.copyOf(inT);
	}

	public FuzzingCondition dup() {
		return new FuzzingCondition(copyTree(stringTree));
	}

	public String csvInternal(Tree<String> t) {
		String s = "[";
		s = s + t.content() + "|";
		for (int i = 0; i < t.nChildren(); i++) {
			s = s + csvInternal(t.child(i));
		}
		;
		s = s + "]";
		return s;
	}

	public String csvPrint() {
		return csvInternal(stringTree);
	}

	public String jsonPrint() {
		return FuzzingConditionJSONUtils.conditionToJSONString(stringTree);
	}

	public static Tree<String> parseElement(String s, Tree<String> parent) {
		System.out.println("parseElement:" + s);
		if (s.length() > 0) {
			// Take off the initial and final square brackets
			String inner = s.substring(1, s.length() - 1);
			String[] contents = inner.split("\\|");
			String name = contents[0];
			Tree<String> t = new Tree<String>(name, parent);
			for (int i = 1; i < contents.length; i++) {
				t.addChild(parseElement(contents[i], t));
			}
			return t;
		} else {
			return new Tree<String>("", null);
		}
	}

	public static FuzzingCondition parseCSVString(String startSpec) {
		Tree<String> t = FuzzingConditionJSONUtils.conditionFromJSONString(startSpec);
		return new FuzzingCondition(t);
	}

	public void validateCondition() throws InvalidCondition {
		// Check there is no variable on the same side of a comparison e.g. X<X
		if (elementTree != null) {
			try {
				elementTree.validate();
			} catch (InvalidFuzzingConditionElement e) {
				throw new InvalidCondition(e);
			}
		}
		
		// TODO: Check a composite condition is not always trivially true e.g. X<2 AND X>2
	}
}
