package carsspecific.ros.carsqueue;

import atlasdsl.*;
import carsspecific.moos.carsqueue.MOOSEvent;
import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import middleware.core.*;
import middleware.core.CARSVariableUpdate.VariableInvalid;

public class ROSEventQueue extends CARSLinkEventQueue<MOOSEvent> {

	private final boolean DEBUG_PRINT_DESERIALISED_MSGS = false;
	private final boolean ALWAYS_REQUEST_CLASSIFICATION = true;
	private final String ROS_HOSTNAME = "localhost";
	
	// TODO: move this to the core? ROSATLASCore
	private Ros ros;

	private static final long serialVersionUID = 1L;
	// TODO: shift into parent class
	private ActiveMQProducer outputToCI;
	// 

	public ROSEventQueue(ATLASCore core, Mission mission, int queueCapacity) {
		super(core, queueCapacity, '.');

	}

	public void run() {
		super.run();
	}

	public void registerAfter() {

	}

	public void handleEventSpecifically(MOOSEvent e) {

	}
	
	
	public void setup() {
		ros = new Ros(ROS_HOSTNAME);
		System.out.println("ROS object created");
		ros.connect();
		System.out.println("ROS connect done");
		CARSLinkEventQueue rosQueue = this;

//		Topic echo = new Topic(ros, "/echo", "std_msgs/String");
//		edu.wpi.rail.jrosbridge.messages.Message toSend = new Message("{\"data\": \"hello, world!\"}");
//		echo.publish(toSend);

		
		// TODO: iterate over the robots in the DSL, set up the subscriptions
		// Here just manually encode a velocity subscription
		Topic testVal = new Topic(ros, "/uav_1", "vel");
		
		testVal.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				System.out.println("From ROSbridge: " + message.toString());
				CARSVariableUpdate cv;
				try {
					// TODO: check formatting of raw messages to parse them
					cv = new CARSVariableUpdate("uav_1", "VEL=TEST", core.getTime());
					MOOSEvent e = (MOOSEvent)cv;
					rosQueue.add(e);
				} catch (VariableInvalid e) {
					e.printStackTrace();
				}
			}
		});

//		Service addTwoInts = new Service(ros, "/add_two_ints", "rospy_tutorials/AddTwoInts");
//
//		ServiceRequest request = new ServiceRequest("{\"a\": 10, \"b\": 20}");
//		ServiceResponse response = addTwoInts.callServiceAndWait(request);
//		System.out.println(response.toString());
//		ros.disconnect();
	}
	
	public void close() {
		ros.disconnect();
	}
}
