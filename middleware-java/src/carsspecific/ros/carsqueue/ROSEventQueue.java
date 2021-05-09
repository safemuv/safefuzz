package carsspecific.ros.carsqueue;

import javax.json.*;

import atlasdsl.*;
import atlassharedclasses.GPSPositionReading;
import atlassharedclasses.Point;
import carsspecific.ros.carsqueue.ROSTopicUpdate.ATLASTag;
import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import middleware.core.*;

public class ROSEventQueue extends CARSLinkEventQueue<ROSEvent> {

	private final boolean DEBUG_PRINT_RAW_MESSAGE = false;
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
			if (rtu.tagEquals(ATLASTag.POSE)) {
				JsonObject j = rtu.getJSON();
				JsonObject pose = j.getJsonObject("pose");
				JsonObject pos = pose.getJsonObject("position");
				JsonNumber jx = pos.getJsonNumber("x");
				JsonNumber jy = pos.getJsonNumber("y");
				JsonNumber jz = pos.getJsonNumber("z");
				// Convert to ATLAS point
				Point p = new Point(jx.doubleValue(), jy.doubleValue(), jz.doubleValue());
				System.out.println("ATLAS Point:" + p.toString());
				GPSPositionReading gps = new GPSPositionReading(p, 0.0, rtu.getVehicleName());
				core.notifyPositionUpdate(gps);
			}	
			
			if (rtu.tagEquals(ATLASTag.VELOCITY)) {
				JsonObject j = rtu.getJSON();
				JsonObject tw = j.getJsonObject("twist");
				JsonObject linear = tw.getJsonObject("linear");
				JsonNumber jx = linear.getJsonNumber("x");
				JsonNumber jy = linear.getJsonNumber("y");
				JsonNumber jz = linear.getJsonNumber("z");
				Point vel = new Point(jx.doubleValue(), jy.doubleValue(), jz.doubleValue());
				System.out.println("Vel:" + vel.toString());
			}
		}
	}
	
	private void standardSubscribe(String vehicleName, ATLASTag tag, String topicName, Topic t) {
		ROSEventQueue rosQueue = this;
		t.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				if (DEBUG_PRINT_RAW_MESSAGE) {
					System.out.println("From ROSbridge tagged: " + tag.toString() + ":" + message.toString());
				};
				ROSEvent rev = new ROSTopicUpdate(vehicleName, tag, topicName, message, core.getTime());
				rosQueue.add(rev);
			}
		});
	}
	
	private void subscribeForVehicleTopics(String vehicleName) {
		String velTopicName = "/" + vehicleName + "/ual/velocity";
		String posTopicName = "/" + vehicleName + "/ual/pose";
		Topic vel = new Topic(ros, velTopicName, "geometry_msgs/TwistStamped");
		Topic pos = new Topic(ros, posTopicName, "geometry_msgs/PoseStamped");
		standardSubscribe(vehicleName, ATLASTag.VELOCITY, velTopicName, vel);
		standardSubscribe(vehicleName, ATLASTag.POSE, posTopicName, pos);
	}
	
	public void __debugTestSend() {
		Topic echo = new Topic(ros, "/echo_from_atlas", "std_msgs/String");
		Message toSend = new Message("{\"data\": \"hello from ATLAS middleware!\"}");
		echo.publish(toSend);
	}
	
	public void setup() {
		ros = new Ros(ROS_HOSTNAME, 8080,  JRosbridge.WebSocketType.ws);
		System.out.println("ROS object created");
		ros.connect();
		System.out.println("ROS connect done");
		
		__debugTestSend();

		// Iterate over all the robots in the DSL, set up subscriptions for position and velocity
		for (Robot r : mission.getAllRobots()) {
			subscribeForVehicleTopics(r.getName());
		}
	}
	
	public void close() {
		ros.disconnect();
	}
}