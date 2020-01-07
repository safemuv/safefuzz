package moosmapping;

public class UFldNodeBrokerProcess extends MOOSProcess {
	public UFldNodeBrokerProcess(MOOSCommunity parent) {
		super("uFldNodeBroker", parent);
		
		resetProperty("AppTick", 1);
		resetProperty("CommsTick", 1);
		
		// TODO: this should look up the shoreside host from the community
		setProperty("TRY_SHORE_HOST", "pshare_route=localhost:9300");
		setProperty("BRIDGE", "src=VIEW_POLYGON");
		setProperty("BRIDGE", "src=VIEW_POINT");
		setProperty("BRIDGE", "src=VIEW_SEGLIST");
		setProperty("BRIDGE", "src=APPCAST");
		setProperty("BRIDGE", "src=NODE_REPORT_LOCAL,  alias=NODE_REPORT");
		setProperty("BRIDGE", "src=NODE_MESSAGE_LOCAL, alias=NODE_MESSAGE");

		// TODO: these should be setup when sonar sensor is present 
		// Bridges from Vehicle to Shoreside - in uFldNodeBroker configuration
		//bridge =  src=UHZ_SENSOR_CONFIG
		//bridge =  src=UHZ_CONFIG_REQUEST
		//bridge =  src=UHZ_SENSOR_REQUEST
		//bridge =  src=HAZARDSET_REPORT
	}
}