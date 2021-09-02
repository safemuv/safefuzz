package fuzzingengine;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;
import java.util.stream.Stream;

import fuzzingengine.conditionelements.FuzzingConditionElement;
import it.units.malelab.jgea.representation.tree.Tree;
import middleware.core.ATLASCore;

public class FuzzingCondition {
	private Tree<FuzzingConditionElement> specTree;
	
	public FuzzingCondition(Tree<FuzzingConditionElement> specTree) {
		this.specTree = specTree; 
	}

	public Tree<FuzzingConditionElement> getTree() {
		return specTree;
	}
	
	public boolean evaluate(ATLASCore core) {
		FuzzingConditionElement e = specTree.content();
		System.out.println("evaluateTree - " + e + " - " + e.getClass().getCanonicalName());
		
		Object res = e.evaluate(core);
		if (res instanceof Boolean) {
			Boolean resB = (Boolean)res;
			return resB;
		} else {
			// TODO: maybe setup exception here
			return false;
		}
	}
	
	public <T> Tree<T> copyTree(Tree<T> inT) {
		return Tree.copyOf(inT);
	}

	public FuzzingCondition dup() {
		return new FuzzingCondition(copyTree(specTree));
	}

	public String csvPrint() {
		final ByteArrayOutputStream baos = new ByteArrayOutputStream();
	    final String utf8 = StandardCharsets.UTF_8.name();
	    try (PrintStream ps = new PrintStream(baos, true, utf8)) {
	        specTree.prettyPrintLine(ps);
	    } catch (UnsupportedEncodingException e) {
			e.printStackTrace();
		}
	    String data;
		try {
			data = baos.toString(utf8);
			return data;
		} catch (UnsupportedEncodingException e) {
			e.printStackTrace();
			return "";
		}
	}

	public static FuzzingCondition parseCSVString(String startSpec) {
		Tree<FuzzingConditionElement> t = new Tree<FuzzingConditionElement>(null, null);
		return new FuzzingCondition(t);
	}
}

