package fuzzingengine.exptgenerator;

import atlasdsl.Mission;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.FuzzingTimeSpecification;

public abstract class FuzzingTimeSpecificationGenerator {
	protected Mission mission;
	
	public FuzzingTimeSpecificationGenerator(Mission mission) {
		this.mission = mission;
	}

	public abstract FuzzingTimeSpecification gen(VariableSpecification var);
}
