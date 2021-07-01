package fuzzingengine.exptgenerator;

public class ListHasNoElement extends Exception {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private String what;
	
	public ListHasNoElement(String what) {
		this.what = what;
	}
}
