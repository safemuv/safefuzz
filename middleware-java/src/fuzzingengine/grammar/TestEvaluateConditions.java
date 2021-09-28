package fuzzingengine.grammar;

import java.util.HashMap;
import java.util.Map;

import fuzzingengine.UnrecognisedBinOp;
import fuzzingengine.UnrecognisedComparison;
import fuzzingengine.UnrecognisedTreeNode;
import fuzzingengine.UnrecognisedUnOp;
import fuzzingengine.grammar.conditionelements.FuzzingConditionElement;
import it.units.malelab.jgea.representation.tree.Tree;
import middleware.core.ObjectLambda;

public class TestEvaluateConditions {
	GrammarConvertorFixedVarFunctions gconv;
	
	private double MIN_X = 0.0;
	private double MAX_X = 10.0;
	private double MIN_Y = 0.0;
	private double MAX_Y = 10.0;
	private double MIN_Z = 0.0;
	private double MAX_Z = 10.0;
	
	private double STEP = 0.1;
	
	TestEvaluateConditions() {
		Map<String,ObjectLambda> fmap = new HashMap<String,ObjectLambda>();
		gconv = new GrammarConvertorFixedVarFunctions(fmap);
	}
	
	// Evaluates it under the given range of variables
	public void testEvaluateCondition(Tree<String> stringTree, String vehicle) {
		try {
			// Loop over all the variable ranges
			for (double x = MIN_X; x <MAX_X; x+=STEP) {
				for (double y = MIN_Y; y < MAX_Y; y+=STEP) {
					for (double z = MIN_Z; z < MAX_Z; z+=STEP) {
						Map<String,ObjectLambda> fmap = new HashMap<String,ObjectLambda>();
						
						// Set left_wing_dist func
						// Set right_wing_dist func
						// Set airframe_distance func
						
						gconv.updateFunctions(fmap);
						FuzzingConditionElement cElt = gconv.convert(stringTree);
						Object res = cElt.evaluate(null, vehicle);
						if (res instanceof Boolean) {
							Boolean r = (Boolean)res;
							if (r) {
								// Log as true
							} else {
								// Log as false
							}
						}
					}
				}
			}
			FuzzingConditionElement cElt = gconv.convert(stringTree);
			
			// Check if the boolean is true or false
		} catch (UnrecognisedComparison | UnrecognisedTreeNode | UnrecognisedUnOp | UnrecognisedBinOp e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
