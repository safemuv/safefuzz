package middleware.gui;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.swing.*;
import atlasdsl.*;
import atlassharedclasses.Point;
import faultgen.FaultGenerator;

public class GUITest {  
	JFrame f;
    HashMap<Robot,JLabel> robotLabels = new LinkedHashMap<Robot, JLabel>();
    private Mission mission;
    private FaultButtonListener l = new FaultButtonListener();
    private FaultGenerator faultGen;
	
	private class FaultButtonListener implements ActionListener {
		public void actionPerformed(ActionEvent arg0) {
			double testTimeLength = 20.0;
			faultGen.injectDynamicFaultNow(testTimeLength);
		}
	}	
   
    private void setupLabels() {
    	int y = 0;
    	for (Robot r : mission.getAllRobots()) {
	    	JLabel l = new JLabel(labelText(r));
	    	l.setVisible(true);
	    	robotLabels.put(r, l);
	    	f.add(l);
    	}
    }
    
    private String labelText(Robot r) {
    	try {
    		Point loc = r.getPointComponentProperty("location");
    		return r.getName() +  " @ " + loc.toString();
    	} catch (MissingProperty e) {
    		return r.getName() + "<missing location>";
    	}
    }
    
    private void updateLabels() {
    	for (Map.Entry<Robot, JLabel> entry : robotLabels.entrySet()) {
    		JLabel l = entry.getValue();
    		Robot r = entry.getKey();
    		String text = labelText(r);
    		l.setText(text);
    		l.repaint();
    	}
    }
    
    public GUITest(Mission mission, FaultGenerator faultGen) {
    	this.mission = mission;
    	this.faultGen = faultGen;
    	
    	f=new JFrame();//creating instance of JFrame
    	f.setLayout(new FlowLayout(FlowLayout.RIGHT));
    	setupLabels();
              
    	JButton b=new JButton("Inject overspeed fault");//creating instance of JButton  
    	b.setBounds(130,100,100, 40);
    	
    	// TODO: need a drop-down to select the robots to inject
        b.addActionListener(l);
    	
    	f.add(b);
    	
    	
    	f.setSize(200,500);//400 width and 500 height  
    	f.setVisible(true);//making the frame visible
    	f.repaint();
    }
    
    public synchronized void updateGUI() {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
    			updateLabels();
    			f.repaint();
			}
		});
    }
}
