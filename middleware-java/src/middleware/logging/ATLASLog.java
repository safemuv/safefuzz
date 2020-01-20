package middleware.logging;

public class ATLASLog {
	// TODO: logger with singleton? 
	public static synchronized void logActiveMQInbound(String queueName, String text) {
        System.out.println("ActiveMQConsumer.handleMessage on " + queueName + " received textMessage: " + text);
	}
}
