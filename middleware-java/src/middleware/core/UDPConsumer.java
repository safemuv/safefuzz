package middleware.core;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import middleware.logging.DebugLog;

public abstract class UDPConsumer implements Runnable {
	private DatagramSocket datagramSocket;
	protected ATLASEventQueue carsQueue;
	
	private static final boolean DEBUG_DISPLAY_UDP_BUFFER = true;
	
	private final int BYTE_BUFFER_LENGTH = 65535;

	private boolean listenCont = true;
	private byte[] receive = new byte[BYTE_BUFFER_LENGTH];
	private int port; 
   
	public UDPConsumer(ATLASEventQueue carsQueue, int port) {
		this.port = port;
		this.carsQueue = carsQueue;
	}
	
	public void run() {
		try {
			datagramSocket = new DatagramSocket(port);
			DatagramPacket DpReceive = null; 
			System.out.println("UDP consumer: creating datagram socket on " + port);
		    while (listenCont) { 
		        DpReceive = new DatagramPacket(receive, receive.length); 
		        datagramSocket.receive(DpReceive); 
		        int receivedLen = DpReceive.getLength();
		        // TODO: push UDP events into a corresponding ATLASEventQueue
		        handleUDPMesssage(receive, receivedLen);
		        if (DEBUG_DISPLAY_UDP_BUFFER) {
		        	System.out.println("UDP Reception Buffer:" + DebugLog.bytesToHex(receive, receivedLen)); 
		        }
		        receive = new byte[BYTE_BUFFER_LENGTH]; 
		    } 			
		} catch (SocketException e) {
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

    protected abstract void handleUDPMesssage(byte[] receive, int len);

	public synchronized void stop() {
    	listenCont = false;
    }
}