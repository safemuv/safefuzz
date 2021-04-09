package atlascollectiveintgenerator.runner;

import java.lang.reflect.InvocationTargetException;

import atlascollectiveint.custom.*;
import atlascollectiveintgenerator.CollectiveInt;

public class CollectiveIntRunner {
	private static final String DEFAULT_CLASS_NAME = "atlascollectiveint.custom.ComputerCIshoreside";
	
	public static void main(String [] args) {
		
		String cIClassName = DEFAULT_CLASS_NAME;
		if (args.length > 0) {
			cIClassName = args[0];
		}
		
		CollectiveInt c;
		try {
			c = new CustomCollectiveInt(cIClassName);
			System.out.println("CustomCollectiveInt created");
			c.init();
			System.out.println("init completed");
			c.startCI();
		} catch (ClassNotFoundException | InstantiationException | IllegalAccessException e) {
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InvocationTargetException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (NoSuchMethodException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (SecurityException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
