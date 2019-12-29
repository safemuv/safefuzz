package moosmapping;

public class PLoggerProcess extends MOOSProcess {
	public PLoggerProcess(MOOSCommunity parent, String loggerFilename, Boolean isShoreside) {
		super("pLogger", parent);
		
		setProperty("AppTick", 10);
		setProperty("CommsTick", 10);
		setProperty("File", loggerFilename);
		
		setProperty("Path", "./");
		setProperty("SyncLog", "true @ 0.2");
		setProperty("AsyncLog", "true");
		setProperty("FileTimeStamp", "true");
		setProperty("WildCardLogging", "true");
		
		if (isShoreside) {
			// Set extra properties here
		} else {
			
		}
	}
}
