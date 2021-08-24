package fuzzingengine;

public abstract class FuzzingTimeSpecification {
	public abstract boolean isActiveAtTime(double atTime);
	public abstract String getCSVContents();
	public abstract String getCSVRecordTag();
	protected abstract FuzzingTimeSpecification dup();
}
