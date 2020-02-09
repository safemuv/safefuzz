package middleware.gui;

import java.awt.FlowLayout;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.swing.*;
import atlasdsl.*;
import atlassharedclasses.Point;

public class GUITest {  
    JFrame f;
    HashMap<Robot,JLabel> robotLabels = new LinkedHashMap<Robot, JLabel>();
    private Mission mission;
    
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
    
    public GUITest(Mission mission) {
    	this.mission = mission;
    	
    	f=new JFrame();//creating instance of JFrame
    	f.setLayout(new FlowLayout(FlowLayout.RIGHT));
    	setupLabels();
              
    	JButton b=new JButton("click");//creating instance of JButton  
    	b.setBounds(130,100,100, 40);  
              
    	f.add(b);//adding button in JFrame  
              
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
