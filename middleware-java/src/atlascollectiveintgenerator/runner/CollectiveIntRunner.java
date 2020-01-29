package atlascollectiveintgenerator.runner;

import atlascollectiveint.custom.*;
import atlascollectiveintgenerator.CollectiveInt;

public class CollectiveIntRunner {
	public static void main(String [] args) {
		CollectiveInt c = new CustomCollectiveInt();
		c.startCI();
	}
}
