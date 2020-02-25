package atlascollectiveintgenerator.runner;

import atlascollectiveint.custom.*;
import atlascollectiveintgenerator.CollectiveInt;

public class CollectiveIntRunner {
	public static void main(String [] args) {
		CollectiveInt c = new CustomCollectiveInt();
		System.out.println("CustomCollectiveInt created");
		c.init();
		System.out.println("init completed");
		c.startCI();
	}
}
