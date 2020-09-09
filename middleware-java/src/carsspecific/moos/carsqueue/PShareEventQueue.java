package carsspecific.moos.carsqueue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;

import activemq.portmapping.PortMappings;
import atlasdsl.*;
import atlassharedclasses.*;
import middleware.core.*;

public class PShareEventQueue extends ATLASEventQueue<PShareEvent> {
	private static final long serialVersionUID = 1L;
	private Pattern detectionScanner;
	private Mission mission;
	private List<UDPConsumer> activeConsumers = new ArrayList<UDPConsumer>();
	private UDPProducer reflectBack;
	
	private final int UDP_RECEPTION_PORT = PortMappings.portNumberForPShareReception();
	private final int UDP_TRANSMISSION_PORT = PortMappings.portNumberForPShareTransmission();
	private final String IP_ADDRESS = PortMappings.addressForPShare();

	public PShareEventQueue(ATLASCore core, Mission mission, int queueCapacity) {
		super(core, queueCapacity, '.');
		this.mission = mission;
		detectionScanner = Pattern.compile("x=([^,]+),y=([^,]+),label=([^,]+),vname=([^,]+)");
		atlasOMapper = new ATLASObjectMapper();
		reflectBack = new UDPProducer(IP_ADDRESS, UDP_TRANSMISSION_PORT);
	}

	public void run() {
		super.run();
	}

	public void setup() {
		UDPConsumer consumer = new PShareUDPConsumer(this, UDP_RECEPTION_PORT);
		activeConsumers.add(consumer);
		startThread(consumer, false);
	}

	public void handleEvent(PShareEvent event) {
		System.out.println("PShareEvent - " + event);
		try {
			reflectBack.sendBack(event);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
