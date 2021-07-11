package fuzzingengine.spec;

import java.util.Optional;
import atlasdsl.Mission;

import fuzzingengine.*;
import fuzzingengine.operationparamsinfo.*;

public class GeneratedFuzzingSpec {

	public static FuzzingEngine createFuzzingEngine(Mission m, boolean loadCSV) {
	FuzzingEngine fe = new FuzzingEngine(m);
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
	
	
	try {
	
	
	   OperationParameterSet ops1 = new OperationParameterSet("JSONPointChange-Fixed-1", "JSONPointChange");
 
 
	   			OperationParameter opp1 = new StringConstantOperationParameter("Nature", "FIXED");
	   			
	   		
	   		
	   		
	   		ops1.addParameter(opp1);
	   			
 
	   			OperationParameter opp2 = new DoubleRangeOperationParameter("X", -1.0, 1.0);
	   		
	   		
	   		
	   		ops1.addParameter(opp2);
	   			
 
	   			OperationParameter opp3 = new DoubleRangeOperationParameter("Y", -1.0, 1.0);
	   		
	   		
	   		
	   		ops1.addParameter(opp3);
	   			
 
	   			OperationParameter opp4 = new DoubleRangeOperationParameter("Z", -1.0, 1.0);
	   		
	   		
	   		
	   		ops1.addParameter(opp4);
	   OperationParameterSet ops2 = new OperationParameterSet("PathPointChange-multiple-elements", "PathPointChanges");
 
 
	   			OperationParameter opp5 = new StringConstantOperationParameter("Nature", "RANDOMOFFSET_MULTIPLE");
	   			
	   		
	   		
	   		
	   		ops2.addParameter(opp5);
	   			
 
	   			OperationParameter opp6 = new DoubleRangeOperationParameter("X", 0.0, 3.0);
	   		
	   		
	   		
	   		ops2.addParameter(opp6);
	   			
 
	   			OperationParameter opp7 = new DoubleRangeOperationParameter("Y", 0.0, 3.0);
	   		
	   		
	   		
	   		ops2.addParameter(opp7);
	   			
 
	   			OperationParameter opp8 = new DoubleRangeOperationParameter("Z", 0.0, 3.0);
	   		
	   		
	   		
	   		ops2.addParameter(opp8);
	   			
	   		
	   		
	   		
 
	   			OperationParameter opp9 = new IntRangeOperationParameter("ELEMENTS_TO_MUTATE", 1, 10);
	   		ops2.addParameter(opp9);
	   OperationParameterSet ops3 = new OperationParameterSet("IntChange-300-3000", "IntegerVariableChange");
 
 
	   			OperationParameter opp10 = new StringConstantOperationParameter("NATURE", "RANDOM");
	   			
	   		
	   		
	   		
	   		ops3.addParameter(opp10);
	   			
	   		
	   		
 
	   			OperationParameter opp11 = new IntConstantOperationParameter("Lower", 300);
	   		
	   		ops3.addParameter(opp11);
	   			
	   		
	   		
 
	   			OperationParameter opp12 = new IntConstantOperationParameter("Upper", 3000);
	   		
	   		ops3.addParameter(opp12);
	
	
	
	
		
		
		
		
	simMapping.setComponentFuzzingInfo("trajectory_tracking_controller_fuzzy", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), false);
		
				
		simMapping.addRecord("trajectory_tracking_controller_fuzzy", "/ual/set_velocity_prime", "/ual/set_velocity", 
		FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv//trajectory_tracking_controller_fuzzy"), Optional.of("geometry_msgs/TwistStamped"),
		true, Optional.of(new TimeSpec(m,0.0,200.0)), Optional.of(1.0));
	
			simMapping.addOperationParameterSetForVariable("/ual/set_velocity_prime", ops1, "twist.linear");
	
		
		
		
		
	simMapping.setComponentFuzzingInfo("CalibrationDefinitions", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), true);
		
				
		simMapping.addRecord("CalibrationDefinitions", "calibration_points.yaml", "", 
		FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv/safemuv_situational_awareness/config"), Optional.empty(),
		false, Optional.empty(), Optional.of(0.0));
	
			simMapping.addOperationParameterSetForVariable("calibration_points.yaml", ops1, "frameB_0");
			simMapping.addOperationParameterSetForVariable("calibration_points.yaml", ops1, "frameB_1");
			simMapping.addOperationParameterSetForVariable("calibration_points.yaml", ops1, "frameB_2");
			simMapping.addOperationParameterSetForVariable("calibration_points.yaml", ops1, "frameB_3");
			simMapping.addOperationParameterSetForVariable("calibration_points.yaml", ops1, "frameB_4");
			simMapping.addOperationParameterSetForVariable("calibration_points.yaml", ops1, "frameB_5");
	
		
		
		
		
	simMapping.setComponentFuzzingInfo("trajectory_planning", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), false);
		
				
		simMapping.addRecord("trajectory_planning", "/desired_trajectory_path_prime", "/desired_trajectory_path", 
		FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv/"), Optional.of("nav_msgs/Path"),
		true, Optional.empty(), Optional.of(0.0));
	
	
				
		simMapping.addRecord("trajectory_planning", "/desired_path_prime", "/desired_path", 
		FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv/"), Optional.of("nav_msgs/Path"),
		true, Optional.of(new TimeSpec(m,0.0,200.0)), Optional.of(0.0));
	
			simMapping.addOperationParameterSetForVariable("/desired_path_prime", ops2, "poses");
	
		
		
		
		
	simMapping.setComponentFuzzingInfo("traj_plan_external", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), true);
		
				
		simMapping.addRecord("traj_plan_external", "trajectory_planner_external.yaml", "", 
		FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv/traj_plan_external"), Optional.empty(),
		false, Optional.empty(), Optional.of(0.0));
	
			simMapping.addOperationParameterSetForVariable("trajectory_planner_external.yaml", ops3, "trajectory_planner_geometric_primitives.discrete_graph.num_nodes");
	
	

	} catch (InvalidTimeSpec its) {
			System.out.println("InvalidTimeSpec - " + its.getNature());
			its.printStackTrace();
	}
	
	fe.setSimMapping(simMapping);
	if (loadCSV) {
		fe.setupFromFuzzingFile("/home/jharbin//academic/atlas/atlas-middleware/fuzz-configs/ros-fuzztest.csv", m);
	}
	return fe;
	}
}