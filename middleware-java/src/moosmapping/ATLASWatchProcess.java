package moosmapping;

public class ATLASWatchProcess extends MOOSProcess {
	public ATLASWatchProcess(MOOSCommunity parent, String vehicleName) {
		super("ATLASDBWatch", parent);
		setProperty("ACTIVEMQ_PORT", 61616);
		// TODO: something global to assign queue names/port names between the middleware and MOOS code generator
		setProperty("ACTIVEMQ_TOPIC", "MIDDLEWARE-watch-" + vehicleName);
		
		// TODO: set up these variables within the code generator
		setProperty("WATCH_VAR", "UHZ_DETECTION_REPORT");
		setProperty("WATCH_VAR", "NODE_REPORT_ELLA");
		setProperty("WATCH_VAR", "NODE_REPORT_FRANK");
		setProperty("WATCH_VAR", "NODE_REPORT_GILDA");
		setProperty("WATCH_VAR", "NODE_REPORT_HENRY");
	}
	
	public void addWatchVariable(String varname) {
		setProperty("WATCH_VAR", varname);
	}
}
