package carsspecific.ros.carsqueue;

import javax.json.JsonObject;

import atlasdsl.*;
import carsspecific.ros.carsqueue.ROSTopicUpdate.ATLASTag;
import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import middleware.core.*;

public class ROSEventQueue extends CARSLinkEventQueue<ROSEvent> {

	private final boolean DEBUG_PRINT_DESERIALISED_MSGS = false;
	private final boolean ALWAYS_REQUEST_CLASSIFICATION = true;
	private final String ROS_HOSTNAME = "localhost";
	private Mission mission;
	
	// TODO: move this to the core? ROSATLASCore
	private Ros ros;

	private static final long serialVersionUID = 1L;
	// TODO: shift into parent class
	private ActiveMQProducer outputToCI;
	// 

	public ROSEventQueue(ATLASCore core, Mission mission, int queueCapacity) {
		super(core, queueCapacity, '.');
		this.mission = mission;
	}

	public void run() {
		super.run();
	}

	public void registerAfter() {

	}

	public void handleEventSpecifically(ROSEvent e) {
		if (e instanceof ROSTopicUpdate) {
			ROSTopicUpdate rtu = (ROSTopicUpdate)e;
			if (rtu.tagEquals(ATLASTag.VELOCITY)) {
				Message m = rtu.getMessage();
				JsonObject j = m.toJsonObject();
				System.out.println("velocity json = " + j);
				
			}	
		}
	}
	
	private void standardSubscribe(String vehicleName, ATLASTag tag, String topicName, Topic t) {
		ROSEventQueue rosQueue = this;
		t.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				System.out.println("From ROSbridge: " + message.toString());
				ROSEvent rev = new ROSTopicUpdate(vehicleName, tag, topicName, message, core.getTime());
				rosQueue.add(rev);
			}
		});
	}
	
	private void subscribeForVehicleTopics(String vehicleName) {
		String velTopicName = "/" + vehicleName + "/ual/velocity";
		String posTopicName = "/" + vehicleName + "/ual/pose";
		Topic vel = new Topic(ros, velTopicName, "geometry_msgs/TwistStamped");
		Topic pos = new Topic(ros, posTopicName, "geometry_msgs/Position");
		standardSubscribe(vehicleName, ATLASTag.VELOCITY, velTopicName, vel);
		standardSubscribe(vehicleName, ATLASTag.POSITION, posTopicName, pos);
	}
	
	
	public void setup() {
		ros = new Ros(ROS_HOSTNAME, 8080,  JRosbridge.WebSocketType.ws);
		System.out.println("ROS object created");
		ros.connect();
		System.out.println("ROS connect done");

		// Iterate over all the robots in the DSL, set up subscriptions for position and velocity
		for (Robot r : mission.getAllRobots()) {
			subscribeForVehicleTopics(r.getName());
		}
	}
	
	public void close() {
		ros.disconnect();
	}
}