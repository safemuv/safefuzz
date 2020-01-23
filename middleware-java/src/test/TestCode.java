package test;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class TestCode {
	private static Pattern nodeReportScanner = Pattern.compile("X=([^,]+),Y=([^,]+)");
	
	public static void testMatch(String test) {
		Matcher m = nodeReportScanner.matcher(test);
		if (m.find()) {
			double x = Double.parseDouble(m.group(1));
			double y = Double.parseDouble(m.group(2));
			System.out.println("x = " + x + " : y = " + y);
		} else {
			System.out.println("Match failed!");
		}
	}
	
	public static void main(String args []) {
		testMatch("X=32.52,Y=-3.132");
		testMatch("X=21,Y=2");
		testMatch("X=2,Y=7,Z=532");
		testMatch("T=121,X=32.52,Y=-4.21");
		testMatch("T=121,X=122.,Y=-4.21");
	}
}