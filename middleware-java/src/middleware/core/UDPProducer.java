package middleware.core;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

import activemq.portmapping.PortMappings;
import carsspecific.moos.carsqueue.PShareEvent;

public class UDPProducer {
	private DatagramSocket socket;
	private byte[] buf;
	private int port;
	private InetAddress addr;

	public UDPProducer(String address, int port) {
		try {
			this.buf = new byte[PortMappings.bufferSizeUDP()];
			this.port = port;
			this.addr = InetAddress.getByName(address);
			this.socket = new DatagramSocket();
		} catch (SocketException e) {
			e.printStackTrace();
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
	}

	public void sendBack(PShareEvent event) throws IOException {
		System.out.println("sendBack - " + event);
		// Copy the contents to the sending buffer
		int len = event.byteLen();
		System.arraycopy(event.byteContents(), 0, buf, 0, len);
		DatagramPacket packet = new DatagramPacket(buf, len, addr, port);
		System.out.println("len = " + len);
		socket.send(packet);
	}
}
