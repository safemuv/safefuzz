package fuzzingengine;

public abstract class FuzzingTimeSpecification {
	public abstract boolean isActiveAtTime(double time, String vehicle);
	public abstract String getCSVContents();
	public abstract String getCSVRecordTag();
	protected abstract FuzzingTimeSpecification dup();
	public abstract void validateSpecification() throws InvalidSpecification;
}
