package test.rosinterface;

import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Service;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;
import edu.wpi.rail.jrosbridge.services.ServiceResponse;

public class ROSConnectivityExample {
	public static void main(String[] args) throws InterruptedException {
		
		// Quick hack for now to fix connectivity: Line 99 in /usr/lib/python2.7/dist-packages/autobahn/websocket/protocol.py
		// Comment out the "if not host": lines 98-100
		// in __is_same_origin: (now line 115) add "return True" at start to skip checks
		
		Ros ros = new Ros("localhost");
		ros.connect();

		Topic echo = new Topic(ros, "/echo", "std_msgs/String");
		Message toSend = new Message("{\"data\": \"hello, world!\"}");
		echo.publish(toSend);

		Topic echoBack = new Topic(ros, "/echo_back", "std_msgs/String");
		echoBack.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				System.out.println("From ROS: " + message.toString());
			}
		});

		Service addTwoInts = new Service(ros, "/add_two_ints", "rospy_tutorials/AddTwoInts");

		ServiceRequest request = new ServiceRequest("{\"a\": 10, \"b\": 20}");
		ServiceResponse response = addTwoInts.callServiceAndWait(request);
		System.out.println(response.toString());
		
		ros.disconnect();
	}
}
