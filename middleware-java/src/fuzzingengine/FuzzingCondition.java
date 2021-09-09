package fuzzingengine;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

import fuzzingengine.conditionelements.*;
import it.units.malelab.jgea.representation.tree.Tree;
import middleware.core.ATLASCore;

public class FuzzingCondition {
	private FuzzingConditionElement elementTree;
	private Tree<String> stringTree;
	private GrammarConvertor conv = new GrammarConvertor();

	private ATLASCore core;

	public FuzzingCondition(Tree<String> stringTree) {
		this.stringTree = stringTree;
	}

	public void doConversion() throws UnrecognisedComparison, UnrecognisedTreeNode {
		this.elementTree = conv.convert(stringTree);
	}

	public Tree<String> getTree() {
		return stringTree;
	}

	public boolean evaluate() {
		if (this.elementTree == null) {
			System.out.println("ELEMENTTREE is null");
			return false;
		} else {
			Object res = this.elementTree.evaluate(core);
			if (res instanceof Boolean) {
				Boolean resB = (Boolean) res;
				return resB;
			} else {
				System.out.println("Returning FALSE - non-boolean returned in evaluating condition " + this);
				return false;
			}
		}
	}

	public <T> Tree<T> copyTree(Tree<T> inT) {
		return Tree.copyOf(inT);
	}

	public FuzzingCondition dup() {
		return new FuzzingCondition(copyTree(stringTree));
	}

//	public String csvPrint() {
//		final ByteArrayOutputStream baos = new ByteArrayOutputStream();
//	    final String utf8 = StandardCharsets.UTF_8.name();
//	    try (PrintStream ps = new PrintStream(baos, true, utf8)) {
//	        stringTree.prettyPrintLine(ps);
//	    } catch (UnsupportedEncodingException e) {
//			e.printStackTrace();
//		}
//	    String data;
//		try {
//			data = baos.toString(utf8);
//			return data;
//		} catch (UnsupportedEncodingException e) {
//			e.printStackTrace();
//			return "";
//		}
//	}

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
		// return new FuzzingCondition(parseElement(startSpec, null));
		Tree<String> t = FuzzingConditionJSONUtils.conditionFromJSONString(startSpec);
		return new FuzzingCondition(t);
	}
}
