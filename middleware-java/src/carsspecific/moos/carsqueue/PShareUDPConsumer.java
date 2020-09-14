package carsspecific.moos.carsqueue;

import middleware.core.ATLASEventQueue;
import middleware.core.UDPConsumer;

public class PShareUDPConsumer extends UDPConsumer {

	public PShareUDPConsumer(ATLASEventQueue<PShareEvent> carsQueue, int port) {
		super(carsQueue, port);
	}

	protected void handleUDPMesssage(byte[] receive, int len) {
		try {
			PShareEvent e = new PShareEvent(receive, len);
			carsQueue.add(e);
		} catch (PShareEventDecodingError e) {
			
		}

	}
}
