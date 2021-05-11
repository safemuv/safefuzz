package carsspecific.ros.carsqueue;

import java.util.Random;

import javax.json.*;

import atlasdsl.*;
import atlassharedclasses.GPSPositionReading;
import atlassharedclasses.Point;
import carsspecific.ros.carsqueue.ROSTopicUpdate.ATLASTag;
import carsspecific.ros.connection.ROSConnection;
import carsspecific.ros.translations.ROSTranslations;
import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import middleware.core.*;

public class ROSEventQueue extends CARSLinkEventQueue<ROSEvent> {

	private final boolean DEBUG_PRINT_RAW_MESSAGE = false;
	private Mission mission;
	private Ros ros;
	
	// Debug test to reflect velocity values directly
	private ROSTranslations __rtrans = new ROSTranslations();
	
	private Random __rng = new Random();

	private static final long serialVersionUID = 1L;
	private static final boolean DEBUG_HACK_TEST_FUZZ_VELOCITY = true;

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
			ROSTopicUpdate rtu = (ROSTopicUpdate) e;
			if (rtu.tagEquals(ATLASTag.POSE)) {
				JsonObject j = rtu.getJSON();
				JsonObject pose = j.getJsonObject("pose");
				JsonObject pos = pose.getJsonObject("position");
				JsonNumber jx = pos.getJsonNumber("x");
				JsonNumber jy = pos.getJsonNumber("y");
				JsonNumber jz = pos.getJsonNumber("z");
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
				
				if (DEBUG_HACK_TEST_FUZZ_VELOCITY) {
					Point randomOffset = new Point(__rng.nextDouble()*2 - 1, __rng.nextDouble()*2 - 1, 0.0);
					__rtrans.setVelocity(rtu.getVehicleName(), vel.add(randomOffset));
				}
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
				}
				;
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

//	public void __debugTestSend(String robotName) {
//		Point velLinear = new Point(10.0, -10.0, 0.0);
//		String topicName = "/" + robotName + "/ual/set_velocity";
//		Topic echo = new Topic(ros, topicName, "geometry_msgs/TwistStamped");
////		Topic echo = new Topic(ros, "/echo_from_atlas", "std_msgs/String");
////		Message toSend = new Message("{\"data\": \"hello from ATLAS middleware!\"}");
//
////			Message toSend = new Message("{\"header\": { \"seq\":0, \"stamp\" : { \"secs\": \"0\", \"nsecs\": \"0\"}, \"frame_id\": \"odom\"}, \"twist\": { \"linear\": { \"x\": 1.0, \"y\": 1.0, \"z\": 0.0 }, \"angular\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0}}}");	
////			echo.publish(toSend);
//
//		JsonObject velUpdate = Json.createObjectBuilder().add("header", Json.createObjectBuilder().add("seq", 0)
//				.add("stamp", Json.createObjectBuilder().add("secs", 0).add("nsecs", 0)).add("frame_id", ""))
//				.add("twist",
//						Json.createObjectBuilder()
//								.add("linear", Json.createObjectBuilder().add("x", 1.0).add("y", -1.0).add("z", 2.0))
//								.add("angular", Json.createObjectBuilder().add("x", 0).add("y", 0).add("z", 0)))
//				.build();
//		Message velUpdate_m = new Message(velUpdate);
//		echo.publish(velUpdate_m);
//	}
//
//	public void __debugTestSendBack(String robotName, Point newVel) {
//		Point velLinear = new Point(10.0, -10.0, 0.0);
//		String topicName = "/" + robotName + "/ual/set_velocity";
//		Topic echo = new Topic(ros, topicName, "geometry_msgs/TwistStamped");
////		Topic echo = new Topic(ros, "/echo_from_atlas", "std_msgs/String");
////		Message toSend = new Message("{\"data\": \"hello from ATLAS middleware!\"}");
//
////			Message toSend = new Message("{\"header\": { \"seq\":0, \"stamp\" : { \"secs\": \"0\", \"nsecs\": \"0\"}, \"frame_id\": \"odom\"}, \"twist\": { \"linear\": { \"x\": 1.0, \"y\": 1.0, \"z\": 0.0 }, \"angular\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0}}}");	
////			echo.publish(toSend);
//
//		JsonObject velUpdate = Json.createObjectBuilder().add("header", Json.createObjectBuilder().add("seq", 0)
//				.add("stamp", Json.createObjectBuilder().add("secs", 0).add("nsecs", 0)).add("frame_id", "odom"))
//				.add("twist",
//						Json.createObjectBuilder()
//								.add("linear", Json.createObjectBuilder().add("x", newVel.getX()).add("y", newVel.getY()).add("z", newVel.getZ()))
//								.add("angular", Json.createObjectBuilder().add("x", 0).add("y", 0).add("z", 0)))
//				.build();
//		Message velUpdate_m = new Message(velUpdate);
//		echo.publish(velUpdate_m);
//	}
	
//	public void __debugSendBack(String robotName, Message source) {
//		String topicName = "/" + robotName + "/ual/set_velocity";
//		Message dest = source.clone();
//		JsonObject j = source.toJsonObject();
//		JsonObject twist = j.getJsonObject("twist");
//		JsonObject linear = twist.getJsonObject("linear");
//		JsonObject linear = new JsonObject();
//	}

	public void setup() {
		ros = ROSConnection.getConnection().getROS();
		// Iterate over all the robots in the DSL, set up subscriptions for position and
		// velocity
		for (Robot r : mission.getAllRobots()) {
			subscribeForVehicleTopics(r.getName());
		}
	}

	public void close() {
		ros.disconnect();
	}
}