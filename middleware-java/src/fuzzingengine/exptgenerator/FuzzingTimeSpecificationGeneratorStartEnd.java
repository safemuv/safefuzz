package fuzzingengine.exptgenerator;

import java.util.Optional;
import java.util.Random;

import atlasdsl.Mission;
import fuzzingengine.FuzzingFixedTimeSpecification;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.TimeSpec;

public class FuzzingTimeSpecificationGeneratorStartEnd extends FuzzingTimeSpecificationGenerator {
	protected Random rng;
	
	public FuzzingTimeSpecificationGeneratorStartEnd(Mission mission, Random rng) {
		super(mission);
		this.rng = rng;
	}

	protected double getStartTime(Optional<TimeSpec> ts_o) {
		double startLimit;

		if (ts_o.isPresent()) {
			startLimit = ts_o.get().getEndTime();
		} else {
			startLimit = mission.getEndTime();
		}

		return startLimit * rng.nextDouble();
	}

	protected double getEndTime(Optional<TimeSpec> ts_o, double startTime) {
		double endLimit;

		if (ts_o.isPresent()) {
			endLimit = ts_o.get().getEndTime();
		} else {
			endLimit = mission.getEndTime();
		}

		double endTime = (endLimit - startTime) * rng.nextDouble() + startTime;
		return endTime;
	}
	
	public FuzzingTimeSpecification gen(VariableSpecification var) {
		double startTime = getStartTime(var.getTimeSpec());
		double endTime = getEndTime(var.getTimeSpec(), startTime);
		return new FuzzingFixedTimeSpecification(startTime, endTime);
	}
}
