package fuzzingengine.grammar;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import atlassharedclasses.Point;
import fuzzingengine.UnrecognisedBinOp;
import fuzzingengine.UnrecognisedComparison;
import fuzzingengine.UnrecognisedTreeNode;
import fuzzingengine.UnrecognisedUnOp;
import fuzzingengine.grammar.conditionelements.FuzzingConditionElement;
import it.units.malelab.jgea.representation.tree.Tree;
import middleware.core.ObjectLambda;

public class TestEvaluateConditions {
	GrammarConvertorFixedVarFunctions gconv;
	
	private double MIN_X = -50.0;
	private double MAX_X = 50.0;
	private double MIN_Y = -50.0;
	private double MAX_Y = 50.0;
	private double MIN_Z = 0.0;
	private double MAX_Z = 15.0;
	
	private double STEP = 1.0;
	
	public TestEvaluateConditions() {
		Map<String,ObjectLambda> fmap = new HashMap<String,ObjectLambda>();
		gconv = new GrammarConvertorFixedVarFunctions(fmap);
	}
	
	private ObjectLambda generateLambdaToPoint(final double xf, final double yf, final double zf, Point fixedPoint) {
		ObjectLambda dlwt = ((String v) -> {
			Point p = new Point(xf,yf,zf);
			return p.distanceTo(fixedPoint);
		});
		return dlwt;
	}
	
	// Evaluates it under the given range of variables
	public void testEvaluateCondition(Tree<String> stringTree, String vehicle, String filename) throws IOException {
		FileWriter outfile = new FileWriter(filename);
		outfile.write("# " + FuzzingConditionJSONUtils.conditionToJSONString(stringTree) + "\n");
		try {
			System.out.print("Evaluating condition");
			// Loop over all the variable ranges
			for (double x = MIN_X; x <MAX_X; x+=STEP) {
				System.out.print(".");
				for (double y = MIN_Y; y < MAX_Y; y+=STEP) {
					for (double z = MIN_Z; z < MAX_Z; z+=STEP) {
						Map<String,ObjectLambda> fmap = new HashMap<String,ObjectLambda>();
														
						fmap.put("distance_to_left_wing_tip", generateLambdaToPoint(x,y,z, new Point(-61.3,-41.2,8.5)));
						fmap.put("distance_to_right_wing_tip", generateLambdaToPoint(x,y,z, new Point(61.3,-41.2,8.5)));
						
						if (vehicle.equals("uav_1")) {
							fmap.put("starting_point_distance", generateLambdaToPoint(x,y,z, new Point(7.0, -2.0, 0.0)));
						}
						
						if (vehicle.equals("uav_2")) {
							fmap.put("starting_point_distance", generateLambdaToPoint(x,y,z, new Point(7.0, 2.0, 0.0)));
						}
						
						gconv.updateFunctions(fmap);
						FuzzingConditionElement cElt = gconv.convert(stringTree);
						Object res = cElt.evaluate(null, vehicle);
						if (res instanceof Boolean) {
							Boolean r = (Boolean)res;
							int outval;
							if (r) {
								outval = 1;
								outfile.write(x + "," + y + "," + z + "," + outval + "\n");
								//System.out.println("x=" + x + ",y=" + y + ",z=" + z + "***** TRUE");
							} else {
								outval = 0;
								//System.out.println("x=" + x + ",y=" + y + ",z=" + z + "FALSE");
							}
						}
					}
				}
			}
			System.out.println();
			outfile.close();
			
			// Check if the boolean is true or false
		} catch (UnrecognisedComparison | UnrecognisedTreeNode | UnrecognisedUnOp | UnrecognisedBinOp e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
