package fuzzingengine.spec;

import java.util.Optional;
import fuzzingengine.*;
import fuzzingengine.operations.*;

public class GeneratedFuzzingSpec {
	public static FuzzingEngine createFuzzingEngine() {
	FuzzingEngine fe = new FuzzingEngine();
	
	System.out.println("uSimMarine - DESIRED_THRUST");
	System.out.println("");
	System.out.println("uSimMarine - DESIRED_RUDDER");
	System.out.println("");
	
	FuzzingOperation nullOp = new NullFuzzingOperation();
	FuzzingOperation thrustFuzz = NumericVariableChangeFuzzingOperation.RandomOffset(-20.0, 20.0);
	FuzzingOperation rudderFuzz = NumericVariableChangeFuzzingOperation.RandomOffset(-50.0, 50.0);
				
	fe.addFuzzingOperation("DESIRED_THRUST", Optional.of("DEZIRED_THRUST"), Optional.of("uSimMarine"), thrustFuzz);
	fe.addFuzzingOperation("DESIRED_RUDDER", Optional.of("DEZIRED_RUDDER"), Optional.of("uSimMarine"), rudderFuzz);
	fe.addFuzzingOperation("DESIRED_ELEVATOR", Optional.of("DEZIRED_ELEVATOR"), Optional.of("uSimMarine"), nullOp);				
	return fe;
	}
}