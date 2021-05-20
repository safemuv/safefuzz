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
	private Point _randomOffset  = new Point(0.0,0.0,0.0);
	private int velEvents = 0;
	
	// Debug test to reflect velocity values directly
	private ROSTranslations __rtrans = new ROSTranslations();
	
	private Random __rng = new Random();

	private static final long serialVersionUID = 1L;
	private static final boolean DEBUG_HACK_TEST_FUZZ_VELOCITY = false;

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
					if (velEvents % 1000 == 0) {
						_randomOffset = new Point(__rng.nextDouble()*2 - 1, __rng.nextDouble()*2 - 1, 0.0);
					}
					
					velEvents++;
					__rtrans.setVelocity(rtu.getVehicleName(), vel.add(_randomOffset));
				}
			}
		}
	}

	private void standardSubscribe(String vehicleName, ATLASTag tag, String topicName, Topic t, String rosType) {
		ROSEventQueue rosQueue = this;
		t.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				if (DEBUG_PRINT_RAW_MESSAGE) {
					System.out.println("From ROSbridge tagged: " + tag.toString() + ":" + message.toString());
				};
				
				ROSEvent rev = new ROSTopicUpdate(vehicleName, tag, topicName, message, core.getTime(), rosType);
				rosQueue.add(rev);
			}
		});
	}

	private void subscribeForVehicleTopics(String vehicleName) {
		String velTopicName = "/ual/velocity";
		String posTopicName = "/ual/pose";
		String velTopicNameFull = "/" + vehicleName + velTopicName;
		String posTopicNameFull = "/" + vehicleName + posTopicName;
		String velType = "geometry_msgs/TwistStamped";
		String posType = "geometry_msgs/PoseStamped";
		Topic vel = new Topic(ros, velTopicNameFull, velType);
		Topic pos = new Topic(ros, posTopicNameFull, posType);
		standardSubscribe(vehicleName, ATLASTag.VELOCITY, velTopicName, vel, velType);
		standardSubscribe(vehicleName, ATLASTag.POSE, posTopicName, pos, posType);
	}

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