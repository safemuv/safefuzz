package activemq.portmapping;

public class PortMappings {
	// TODO: move others into here
	
	public static String portForCI(String vehicleName) {
		return "MIDDLEWARE-CI-" + vehicleName.toString();
	}
	
	public static String portForCIReverse(String vehicleName) {
		return "CI-MIDDLEWARE-" + vehicleName.toString();
	}
	
	public static String portForMOOSWatch(String vehicleName) {
		return "MIDDLEWARE-watch-" + vehicleName.toString();
	}
}
