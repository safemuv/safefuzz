package carsspecific.ros.rosmapping;

public class PShareProcess extends MOOSProcess {
	// TODO: this currently carries the assumption that the shoreside is the
	// first generated process, on port offset zero, this should be looked up via a 
	// mapping in the MOOS community
	public PShareProcess(MOOSCommunity parent, int pSharePortBase, int portOffset) {
		super("pShare", parent);
	}
	
	public void addShoresideInput() {

	}
}
