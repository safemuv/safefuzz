package atlasdsl.faults;

import java.util.AbstractMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class SubFieldSpec {
	private boolean randomSelection = false;
	// TODO: change this to allow min or max field numbers here
	private int fieldStartNum;
	private int fieldRangeLength;
	Random r = new Random();
	
	public SubFieldSpec(int fieldStartNum, int fieldRangeLength, boolean isRandom) {
		this.fieldStartNum = fieldStartNum;
		this.fieldRangeLength = fieldRangeLength;
		this.randomSelection = isRandom;
	}
	
	private int chooseFieldNum(int max) {
		if (randomSelection) {
			return Math.min(max, fieldStartNum + r.nextInt(fieldRangeLength));
		} else return fieldStartNum;
	}
	
	// The integer returned is the chosen index - so it can be stored back later
	public Map.Entry<Integer, Object> extract(List orig) {
		Integer fieldNum = chooseFieldNum(orig.size());
		Object v = orig.get(fieldNum);
		return new AbstractMap.SimpleEntry<Integer, Object>(fieldNum, v);
	}
	
	public Object store(Object part, List orig, int index) {
		orig.set(index, part);
		return orig;
	}
}
