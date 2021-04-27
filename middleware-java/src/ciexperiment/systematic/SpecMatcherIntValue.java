package ciexperiment.systematic;

public class SpecMatcherIntValue extends SpecMatcher {
	private int fieldNum;
	private int value;
	
	SpecMatcherIntValue(int fieldNum, int value) {
		this.fieldNum = fieldNum;
		this.value = value;
	}
	
	public boolean lineMatches(String [] fields) {
		return (Integer.valueOf(fields[fieldNum]) == value);
	}
}
