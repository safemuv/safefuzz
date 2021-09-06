package fuzzingengine;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;
import java.util.stream.Stream;

import fuzzingengine.conditionelements.*;
import fuzzingengine.conditionelements.FuzzingConditionComparison.ComparisonOperation;
import it.units.malelab.jgea.representation.tree.Tree;
import middleware.core.ATLASCore;

public class FuzzingCondition {
	private FuzzingConditionElement elementTree;
	private Tree<String> stringTree;
	
	private ATLASCore core;
	
	private FuzzingConditionElement convert(Tree<String> stringTree) {
		System.out.println("convert - Tree=" + stringTree);
		String s = stringTree.content();
		if (s.equals("<cond>")) {
			System.out.println("COND");
			return new FuzzingConditionElementCond();
		}
		
		if (s.equals("<var>")) {
			System.out.println("VAR");
			String varname = stringTree.child(0).content();
			return new FuzzingConditionVariable(varname);
		}
		
		if (s.equals("<bincomp>")) {
			System.out.println("BINCOMP");
			Tree<String> lhs = stringTree.child(0);
			Tree<String> rhs = stringTree.child(1);			
			return new FuzzingConditionComparison(convert(lhs), convert(rhs), ComparisonOperation.LESS_THAN);
		}
		
		if (s.equals("<expr>")) {
			return convert(stringTree.child(0));
		}
		
		if (s.equals("<int>")) {
			String valStr = stringTree.child(0).content();
			int v = Integer.valueOf(valStr);
			return new FuzzingConditionConstant(v);
		}
		
		return null;
		
	}
	
	public FuzzingCondition(Tree<String> stringTree) {
		this.elementTree = convert(stringTree); 
	}

	public Tree<String> getTree() {
		return stringTree;
	}
	
	public boolean evaluate() {
		Object res = elementTree.evaluate(core);
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
		return new FuzzingCondition(copyTree(stringTree));
	}

	public String csvPrint() {
		final ByteArrayOutputStream baos = new ByteArrayOutputStream();
	    final String utf8 = StandardCharsets.UTF_8.name();
	    try (PrintStream ps = new PrintStream(baos, true, utf8)) {
	        stringTree.prettyPrintLine(ps);
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
		Tree<String> t = new Tree<String>(null, null);
		return new FuzzingCondition(t);
	}
}

