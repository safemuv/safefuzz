package middleware.logging;

import java.io.FileWriter;
import java.io.IOException;

public class ATLASLog {
	private static ATLASLog logger = null;
	private FileWriter carsInboundLog;
	private FileWriter mqOutboundLog;
	
	ATLASLog() {
		try {
			carsInboundLog = new FileWriter("logs/atlasCARSInbound.log");
			mqOutboundLog = new FileWriter("logs/atlasMQOutbound.log");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private static ATLASLog getLog() {
		if (logger == null) {
			logger = new ATLASLog();
		} 
		return logger;
	}
	
	public static synchronized void logCARSInbound(String queueName, String text) {
		try {
			getLog().carsInboundLog.write("ActiveMQConsumer.handleMessage on " + queueName + " received textMessage: " + text + "\n");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static synchronized void logActiveMQOutbound(String queueName, String text) {
		try {
			getLog().mqOutboundLog.write("ActiveMQProducer.send on " + queueName + " received textMessage: " + text + "\n");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void logCIInbound(String queueName, String text) {
		// TODO Auto-generated method stub
		
	}
}
