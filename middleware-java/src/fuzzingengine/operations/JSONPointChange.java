package fuzzingengine.operations;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonNumber;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonStructure;
import javax.json.JsonValue.ValueType;

import java.util.Random;

import atlassharedclasses.Point;

public class JSONPointChange extends JSONFuzzingOperation {
	private final int defaultFixedChange = 0;
	private String fixedChange = Integer.toString(defaultFixedChange);
	
	private enum ChangeMode {
		FIXED,
		OFFSET,
		RANDOM
	}
	
	private ChangeMode mode;
	private Point p; 
	private Random rng;
	
	private JSONPointChange(ChangeMode mode, Point p) {
		this.mode = mode;
		this.p = p;
		this.rng = new Random();
	}
	
	public static JSONPointChange Random(double xmax, double ymax, double zmax) {
		return new JSONPointChange(ChangeMode.RANDOM, new Point(xmax, ymax, zmax));
	}
	
	public static JSONPointChange RandomOffset(double xmax, double ymax, double zmax) {
		return new JSONPointChange(ChangeMode.OFFSET, new Point(xmax, ymax, zmax));
	}
	
	private static JSONPointChange Fixed(double x, double y, double z) {
		return new JSONPointChange(ChangeMode.FIXED, new Point(x,y,z));
	}

	public String getReplacement(String inValue) {
		return fixedChange;
	}
	
	public JsonObject fuzzObject(JsonObject j) {
		JsonObjectBuilder builder = Json.createObjectBuilder();
		JsonNumber jx = j.getJsonNumber("x");
		JsonNumber jy = j.getJsonNumber("y");
		JsonNumber jz = j.getJsonNumber("z");
		// Modified is by default the incoming point from the fuzzing engine
		Point modified = new Point(jx.doubleValue(), jy.doubleValue(), jz.doubleValue());
		
		if (mode == ChangeMode.OFFSET) {
			Point offset = getRandomPoint();
			modified = modified.add(offset);
		}
		
		if (mode == ChangeMode.FIXED) {
			modified = new Point(p.getX(), p.getY(), p.getZ());
		}
		
		if (mode == ChangeMode.RANDOM) {
			modified = getRandomPoint();
		}
		
		builder.add("x", modified.getX());
		builder.add("y", modified.getY());
		builder.add("z", modified.getZ());
		return builder.build();
	}
	
	public JsonArray fuzzArray(JsonArray j) {
		JsonArrayBuilder builder = Json.createArrayBuilder();
		JsonNumber jx = j.getJsonNumber(0);
		JsonNumber jy = j.getJsonNumber(1);
		JsonNumber jz = j.getJsonNumber(2);
		Point modified = new Point(jx.doubleValue(), jy.doubleValue(), jz.doubleValue());
		
		if (mode == ChangeMode.OFFSET) {
			Point offset = getRandomPoint();
			modified = modified.add(offset);
		}
		
		if (mode == ChangeMode.FIXED) {
			modified = new Point(p.getX(), p.getY(), p.getZ());
		}
			
		if (mode == ChangeMode.RANDOM) {
			modified = getRandomPoint();
		}
		
		builder.add(modified.getX());
		builder.add(modified.getY());
		builder.add(modified.getZ());
		return builder.build();
	}
	
	public JsonStructure fuzzTransformMessage(JsonStructure jStr) {
		if (jStr.getValueType() == ValueType.OBJECT) {
			return fuzzObject((JsonObject)jStr);
		} 
		
		if (jStr.getValueType() == ValueType.ARRAY) {
			return fuzzArray((JsonArray)jStr);
		} 
		
		// If nothing else, return it unchanged
		return jStr;
	}

	private Point getRandomPoint() {
		double xlim = p.getX();
		double ylim = p.getY();
		double zlim = p.getZ();
		double x = rng.nextDouble() * xlim*2 - xlim;
		double y = rng.nextDouble() * ylim*2 - ylim;
		double z = rng.nextDouble() * zlim*2 - zlim;
		return new Point(x,y,z);
	}
	
	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		System.out.println("createFromParamString - " + s);
		String fields [] = s.split("\\|");
		System.out.println(fields[0]);
		System.out.println("fields[0] = " + fields[0]);
		if (fields[0].toUpperCase().equals("RANDOM")) {
			double xmax = Double.valueOf(fields[1]);
			double ymax = Double.valueOf(fields[2]);
			double zmax = Double.valueOf(fields[3]);
			return JSONPointChange.Random(xmax, ymax, zmax);
		}
		
		if (fields[0].toUpperCase().equals("RANDOMOFFSET")) {
			double xmax = Double.valueOf(fields[1]);
			double ymax = Double.valueOf(fields[2]);
			double zmax = Double.valueOf(fields[3]);
			return JSONPointChange.RandomOffset(xmax, ymax, zmax);
		}
		
		if (fields[0].toUpperCase().equals("FIXED")) {
			double x = Double.valueOf(fields[1]);
			double y = Double.valueOf(fields[2]);
			double z = Double.valueOf(fields[3]);
			return JSONPointChange.Fixed(x, y, z);
		}
		
		throw new CreationFailed("Invalid parameter string " + s);
	}
}
