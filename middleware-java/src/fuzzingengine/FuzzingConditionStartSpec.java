package fuzzingengine;

public class FuzzingConditionStartSpec extends FuzzingTimeSpecification {
	FuzzingCondition startCond;
	
	boolean started = false;
	
	double endTime;
	
	public boolean isActiveAtTime(double time) {
		if (!started) {
			started = startCond.isActive();
		} else {
			started = (time < endTime);
		}
		return started;
	}
}
