package activemq.portmapping;

public class PortMappings {
	public static String portForCI(String vehicleName) {
		return "MIDDLEWARE-CI-" + vehicleName.toString();
	}
	
	public static String portForCIReverse(String vehicleName) {
		return "CI-MIDDLEWARE-" + vehicleName.toString();
	}
	
	public static String portForMOOSWatch(String vehicleName) {
		return "MIDDLEWARE-watch-" + vehicleName.toString();
	}
	
	public static String portForMOOSDB(String moosCommunityName) {
		return "FAULTS-SIM-TO-ATLAS-targ_" + moosCommunityName + ".moos";
	}
	
	public static String portForMiddlewareFromCI(String vehicleName) {
		return "MIDDLEWARE-FROM-CI_" + vehicleName.toString();
	}
}
