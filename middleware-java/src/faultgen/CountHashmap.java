package faultgen;

import java.util.HashMap;

public class CountHashmap<T> {
	private HashMap<T,Integer> h = new HashMap<T,Integer>();
	public CountHashmap() {
		
	}
	
	public void incrementCount(T t) {
		if (h.get(t) == null) {
			h.put(t,0);
		} else {
			h.put(t, h.get(t) + 1);
		}
	}
	
	public int getCount(T t) {
		return h.get(t);
	}
}
