package fuzzingengine.operations;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonNumber;
import javax.json.JsonObject;
import javax.json.JsonObjectBuilder;
import javax.json.JsonStructure;
import javax.json.JsonValue;
import javax.json.JsonValue.ValueType;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import atlassharedclasses.Point;

public class PathPointChanges extends JSONFuzzingOperation {
	private final int defaultFixedChange = 0;
	private String fixedChange = Integer.toString(defaultFixedChange);

	private boolean useOffset;
	private Point p;
	private Random rng;
	private int countToFuzz;

	private PathPointChanges(boolean useOffset, Point p, int pointCount) {
		this.useOffset = useOffset;
		this.p = p;
		this.rng = new Random();
		this.countToFuzz = pointCount;
	}

	public static PathPointChanges Random(double xmax, double ymax, double zmax, int pointCount) {
		return new PathPointChanges(false, new Point(xmax, ymax, zmax), pointCount);
	}

	public static PathPointChanges RandomOffset(double xmax, double ymax, double zmax, int pointCount) {
		return new PathPointChanges(true, new Point(xmax, ymax, zmax), pointCount);
	}

	public String getReplacement(String inValue) {
		return fixedChange;
	}

	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		System.out.println("createFromParamString - " + s);
		String fields[] = s.split("\\|");
		System.out.println(fields[0]);
		System.out.println("fields[0] = " + fields[0]);
		if (fields[0].toUpperCase().equals("RANDOM")) {
			double xmax = Double.valueOf(fields[1]);
			double ymax = Double.valueOf(fields[2]);
			double zmax = Double.valueOf(fields[3]);
			return PathPointChanges.Random(xmax, ymax, zmax, 1);
		}

		if (fields[0].toUpperCase().equals("RANDOMOFFSET")) {
			double xmax = Double.valueOf(fields[1]);
			double ymax = Double.valueOf(fields[2]);
			double zmax = Double.valueOf(fields[3]);
			return PathPointChanges.RandomOffset(xmax, ymax, zmax, 1);
		}

		if (fields[0].toUpperCase().equals("RANDOMOFFSET_MULTIPLE")) {
			double xmax = Double.valueOf(fields[1]);
			double ymax = Double.valueOf(fields[2]);
			double zmax = Double.valueOf(fields[3]);
			int pointCount = Integer.valueOf(fields[4]);
			return PathPointChanges.RandomOffset(xmax, ymax, zmax, pointCount);
		}

		throw new CreationFailed("Invalid parameter string " + s);
	}

	public JsonArray fuzzPathArray(JsonArray j) {
		// Find a single array and change points elements from it
		int pointCount = j.size();

		if (pointCount > 0) {

			// Generate some random indices to fuzz

			// This array specifies if we should fuzz the given elements
			List<Boolean> shouldFuzz = new ArrayList<Boolean>();

			for (int i = 0; i < pointCount; i++) {
				shouldFuzz.add(false);
			}

			for (int pi = 0; pi < countToFuzz; pi++) {
				// TODO: should ensure these are unique
				Integer indexToFuzz = rng.nextInt(pointCount);
				shouldFuzz.set(indexToFuzz, true);
			}

			JsonArrayBuilder builder = Json.createArrayBuilder();
			for (int i = 0; i < pointCount; i++) {
				if (!shouldFuzz.get(i)) {
					builder.add(j.get(i));
				} else {
					builder.add(modifiedPoseStamped((JsonObject) j.get(i)));
				}
			}
			return builder.build();
		} else {
			return j;
		}


	}

	private JsonObject modifiedPosition(JsonObject jp) {
		JsonObjectBuilder builder = Json.createObjectBuilder();
		Point modified;
		if (useOffset) {
			JsonNumber jx = jp.getJsonNumber("x");
			JsonNumber jy = jp.getJsonNumber("y");
			JsonNumber jz = jp.getJsonNumber("z");
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

	private JsonValue modifiedPose(JsonObject jp) {
		JsonObjectBuilder builder = Json.createObjectBuilder();
		JsonObject modifiedPos = modifiedPosition(jp.getJsonObject("position"));
		builder.add("position", modifiedPos);
		builder.add("orientation", jp.getJsonObject("orientation"));
		return builder.build();
	}

	private JsonValue modifiedPoseStamped(JsonObject jsonValue) {
		JsonObjectBuilder builder = Json.createObjectBuilder();
		builder.add("header", jsonValue.get("header"));
		builder.add("pose", modifiedPose(jsonValue.getJsonObject("pose")));
		return builder.build();

	}

	public JsonStructure fuzzTransformMessage(JsonStructure jStr) {
		if (jStr.getValueType() == ValueType.OBJECT) {
			System.out.println("PathPointChange expects an array of PoseStamped objects");
		}

		if (jStr.getValueType() == ValueType.ARRAY) {
			return fuzzPathArray((JsonArray) jStr);
		}

		// If nothing else, return it unchanged
		return jStr;
	}

	private Point getRandomPoint() {
		double xlim = p.getX();
		double ylim = p.getY();
		double zlim = p.getZ();
		double x = rng.nextDouble() * xlim * 2 - xlim;
		double y = rng.nextDouble() * ylim * 2 - ylim;
		double z = rng.nextDouble() * zlim * 2 - zlim;
		return new Point(x, y, z);
	}
}
