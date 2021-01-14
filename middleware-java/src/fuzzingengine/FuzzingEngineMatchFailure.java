package fuzzingengine;

import java.util.regex.Pattern;

public class FuzzingEngineMatchFailure extends Exception {
	private static final long serialVersionUID = 1L;
	Pattern p;
	String source;
	
	public FuzzingEngineMatchFailure(Pattern p, String source) {
		this.p = p;
		this.source = source;
	}
}
