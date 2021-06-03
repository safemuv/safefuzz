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
	
	private boolean useOffset;
	private Point p; 
	private Random rng;
	
	private JSONPointChange(boolean useOffset, Point p) {
		this.useOffset = useOffset;
		this.p = p;
		this.rng = new Random();
	}
	
	public static JSONPointChange Random(double xmax, double ymax, double zmax) {
		return new JSONPointChange(false, new Point(xmax, ymax, zmax));
	}
	
	public static JSONPointChange RandomOffset(double xmax, double ymax, double zmax) {
		return new JSONPointChange(true, new Point(xmax, ymax, zmax));
	}

	public String getReplacement(String inValue) {
		return fixedChange;
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
		
		throw new CreationFailed("Invalid parameter string " + s);
	}

	public JsonObject fuzzObject(JsonObject j) {
		JsonObjectBuilder builder = Json.createObjectBuilder();
		Point modified;
		if (useOffset) {
			JsonNumber jx = j.getJsonNumber("x");
			JsonNumber jy = j.getJsonNumber("y");
			JsonNumber jz = j.getJsonNumber("z");
			modified = new Point(jx.doubleValue(), jy.doubleValue(), jz.doubleValue());
			Point offset = getRandomPoint();
			modified = modified.add(offset);
			
		} else { 
			modified = getRandomPoint();
		}
		
		builder.add("x", modified.getX());
		builder.add("y", modified.getY());
		builder.add("z", modified.getZ());
		return builder.build();
	}
	
	public JsonArray fuzzArray(JsonArray j) {
		JsonArrayBuilder builder = Json.createArrayBuilder();
		Point modified;
		if (useOffset) {
			JsonNumber jx = j.getJsonNumber(0);
			JsonNumber jy = j.getJsonNumber(1);
			JsonNumber jz = j.getJsonNumber(2);
			modified = new Point(jx.doubleValue(), jy.doubleValue(), jz.doubleValue());
			Point offset = getRandomPoint();
			modified = modified.add(offset);
			
		} else { 
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
}
