package atlassharedclasses;

public class ATLASTimeUpdate extends ATLASSharedMsg {
	private double time;
	
	public ATLASTimeUpdate() {
		
	}
	
	public ATLASTimeUpdate(double time) {
		this.time = time;
	}	
	
	public double getTime() {
		return time;
	}
}
