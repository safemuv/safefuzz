package moosmapping;

public class USimMarineProcess extends MOOSProcess {
	public USimMarineProcess(MOOSCommunity parent) {
		super("uSimMarine", parent);
		setProperty("START_POS", "0,0,180,0");
		setProperty("PREFIX", "NAV");
	}
}
