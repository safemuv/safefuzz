package atlascollectiveint.logging;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;

public class CollectiveIntLog {
	private static CollectiveIntLog logger = null;
	private FileWriter algorithmLog;
	private FileWriter messagesLog;
	private FileWriter generationLog;
	private FileWriter exceptionLog;
	
	CollectiveIntLog() {
		try {
			algorithmLog = new FileWriter("logs/CIAlgorithm.log");
			messagesLog = new FileWriter("logs/CIMessages.log");
			generationLog = new FileWriter("logs/CIGeneration.log");
			exceptionLog = new FileWriter("logs/CIException.log");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private static CollectiveIntLog getLog() {
		if (logger == null) {
			logger = new CollectiveIntLog();
		} 
		return logger;
	}
	
	public static synchronized void logCI(String text) {
		try {
			getLog().algorithmLog.write(text + "\n");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static synchronized void logCIMessage(String queueName, String text) {
		try {
			getLog().messagesLog.write("ActiveMQProducer.send on " + queueName + " received textMessage: " + text + "\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public static synchronized void logCIGeneration(String queueName, String text) {
		try {
			getLog().generationLog.write("ActiveMQProducer.send on " + queueName + " received textMessage: " + text + "\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public static synchronized void logCIExceptions(Exception expt) {
		try {
			StringWriter sw = new StringWriter();
			expt.printStackTrace(new PrintWriter(sw));
			getLog().exceptionLog.append(sw.toString());
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}