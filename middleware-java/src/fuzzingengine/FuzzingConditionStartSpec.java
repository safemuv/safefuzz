package fuzzingengine;

public class FuzzingConditionStartSpec extends FuzzingTimeSpecification {
	FuzzingCondition startCond;
	
	boolean started = false;
	
	double endTime;
	
	public FuzzingConditionStartSpec(FuzzingCondition startCond, double endTime) {
		this.startCond = startCond.dup();
		this.endTime = endTime;
	}
	
	public boolean isActiveAtTime(double time) {
		if (!started) {
			started = startCond.isActive();
		} else {
			started = (time < endTime);
		}
		return started;
	}

	public String getCSVContents() {
		// TODO Auto-generated method stub
		return null;
	}

	protected FuzzingTimeSpecification dup() {
		return new FuzzingConditionStartSpec(startCond, endTime);
	}
}
