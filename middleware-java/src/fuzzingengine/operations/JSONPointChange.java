package fuzzingengine.operations;

import java.util.Optional;
import java.util.Random;
import javax.json.JsonObject;
import fuzzingengine.IntLambda;

public class JSONPointChange extends JSONFuzzingOperation {
	private final int defaultFixedChange = 0;
	private String fixedChange = Integer.toString(defaultFixedChange);
	private Optional<IntLambda> generateInt = Optional.empty();
	
	public JSONPointChange(IntLambda generateInt) {
		this.generateInt = Optional.of(generateInt);
	}
	
	public JSONPointChange(int fixedChange) {
		this.fixedChange = Integer.toString(fixedChange);
	}
	
	public static JSONPointChange Random(int lower, int upper) {
		Random r = new Random();
		int diff = upper - lower;
		JSONPointChange op = new JSONPointChange(input -> lower + (int)Math.floor(diff * r.nextDouble()));
		return op;
	}
	
	public static JSONPointChange RandomOffset(int lower, int upper) {
		Random r = new Random();
		int diff = upper - lower;
		JSONPointChange op = new JSONPointChange(input -> input + (int)Math.floor(lower + (diff * r.nextDouble())));
		return op;
	}

	public String getReplacement(String inValue) {
		return fixedChange;
	}

	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		String fields [] = s.split("\\|");
		System.out.println(fields[0]);
		if (fields[0].toUpperCase().equals("RANDOM")) {
			int l = Integer.valueOf(fields[1]);
			int r = Integer.valueOf(fields[2]);
			return JSONPointChange.Random(l,r);
		}
		
		throw new CreationFailed("Invalid parameter string " + s);
	}

	public JsonObject fuzzTransformMessage(JsonObject msg) {
		return null;
	}
}
