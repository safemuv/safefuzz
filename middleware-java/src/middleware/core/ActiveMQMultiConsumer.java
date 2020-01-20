package middleware.core;

import java.util.ArrayList;
import java.util.List;

import javax.jms.Connection;
import javax.jms.Destination;
import javax.jms.ExceptionListener;
import javax.jms.JMSException;
import javax.jms.Message;
import javax.jms.MessageConsumer;
import javax.jms.Session;
import javax.jms.TextMessage;

// TODO: make this a multiconsumer that can listen on all the necessary topics for the
// various vehicles
import org.apache.activemq.ActiveMQConnectionFactory;

public class ActiveMQMultiConsumer implements Runnable, ExceptionListener {
	private List<String> queueNames;
	private List<Thread> listeningThreads = new ArrayList<Thread>();
	
	public ActiveMQMultiConsumer(List<String> queueNames) {
		this.queueNames = queueNames;
	}
	
    public void handleMessage(Message m, MessageConsumer c) {
        try {
            if (m instanceof TextMessage) {
                TextMessage textMessage = (TextMessage) m;
                String text = textMessage.getText();
                System.out.println("Received: " + text);
                System.out.println("ActiveMQConsumer.handleMessage received textMessage: " + text);
                // TODO: Push messages to a synchronized central queue?
            } else {
                System.out.println("Received: " + m);
            }
        } catch (Exception e) {
            System.out.println("handleMessage caught: " + e);
            e.printStackTrace();
        }
    }
    
    public void launchThreadedListener(Session session, String queueName) throws JMSException {
    	 // Create the destination (Topic or Queue)
        // TODO: decide an appropriate naming scheme for the queues
        Destination destination = session.createQueue(queueName);

        // Create a MessageConsumer from the Session to the Topic or Queue
        MessageConsumer consumer = session.createConsumer(destination);

        // Loop on new messages
        Thread listener = new Thread(() -> {
        		Message message;
				try {
					boolean threadContinue = true;
					// TODO: some way to exit here under external control
					while (threadContinue) {
						message = consumer.receive();
						handleMessage(message, consumer);
					}
					consumer.close();
				} catch (JMSException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}	
        });
        listeningThreads.add(listener);
        System.out.println("Added listening thread on " + queueName);
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
            
            for (String q : queueNames) {
            	launchThreadedListener(session, q);
            }
            session.close();
            connection.close();
        } catch (Exception e) {
            System.out.println("Caught: " + e);
            e.printStackTrace();
        }
    }

    public synchronized void onException(JMSException ex) {
        System.out.println("JMS Exception occured.  Shutting down client.");
    }
}

