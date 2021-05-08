package carsspecific.ros.carsqueue;

import edu.wpi.rail.jrosbridge.messages.Message;

public class ROSTopicUpdate extends ROSEvent {
	private String vehicleName;
	private String topicName;
	private Message message;
	private double time;
	private ATLASTag atlasTag;
	
	public enum ATLASTag {
		VELOCITY,
		POSITION
	}
	
	public ROSTopicUpdate(String vehicleName, ATLASTag atlasTag, String topicName, Message message, double time) {
		this.vehicleName = vehicleName;
		this.topicName = topicName;
		this.message = message;
		this.time = time;
		this.atlasTag = atlasTag;
	}
	
	public boolean tagEquals(ATLASTag t) {
		return atlasTag.equals(t);
	}
	
	public Message getMessage() {
		return message;
	}
}
