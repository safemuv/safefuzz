package atlascollectiveintgenerator;

import java.util.Optional;

import javax.jms.Connection;
import javax.jms.Destination;
import javax.jms.ExceptionListener;
import javax.jms.JMSException;
import javax.jms.Message;
import javax.jms.MessageConsumer;
import javax.jms.Session;
import javax.jms.TextMessage;

import org.apache.activemq.ActiveMQConnectionFactory;

import atlasdsl.Mission;
import middleware.core.CARSEventQueue;
import middleware.core.MOOSEvent;
import middleware.core.MOOSVariableUpdate;
import middleware.core.SensorDetection;
import middleware.logging.ATLASLog;

public class CollectiveIntActiveMQConsumer implements Runnable, ExceptionListener {
	private String vehicleName;
	private boolean mqListen = true;
	private int pollInterval = 1000;
	
	Mission mission;
	
	public CollectiveIntActiveMQConsumer(String vehicleName, Mission mission) {
		this.vehicleName = vehicleName;
		this.mission = mission;
	}
	
    public void handleMessage(Message m) {
        try {
            if (m instanceof TextMessage) {
                TextMessage textMessage = (TextMessage) m;
                String text = textMessage.getText();
                Optional<SensorDetection> d = SensorDetection.parse(text, mission);
                if (d.isPresent()) {
                	System.out.println("");
                	// TODO: call relevant hooks for the sensor detection here
                }
                
                CollectiveIntLog.logCIInbound(vehicleName, text);
                
            } else {
                System.out.println("Received: " + m);
            }
        } catch (Exception e) {
            System.out.println("handleMessage caught: " + e);
            e.printStackTrace();
        }
    }

    public void run() {
        try {
            // Create a ConnectionFactory
        	ActiveMQConnectionFactory connectionFactory = new ActiveMQConnectionFactory("failover:(tcp://localhost:61616)");
            // Create a Connection
            Connection connection = connectionFactory.createConnection();
            connection.start();

            connection.setExceptionListener(this);

            // Create a Session
            Session session = connection.createSession(false, Session.AUTO_ACKNOWLEDGE);

            // Create the destination (Topic or Queue)
            // TODO: decide an appropriate scheme for the queues - Should we use Topics or Queues for ActiveMQ connections?
            Destination destination = session.createTopic(vehicleName);

            // Create a MessageConsumer from the Session to the Topic or Queue
            MessageConsumer consumer = session.createConsumer(destination);

            // Wait for a message while the thread is active
			while (mqListen) {
				Message message = consumer.receive(pollInterval);
				if (message != null)
					handleMessage(message);
			}

            consumer.close();
            session.close();
            connection.close();
        } catch (Exception e) {
            System.out.println("Caught: " + e);
            e.printStackTrace();
        }
    }
    
    public synchronized void stop() {
    	mqListen = false;
    }

    public synchronized void onException(JMSException ex) {
        System.out.println("JMS Exception occured.  Shutting down client.");
    }
}
