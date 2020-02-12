package middleware.gui;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javax.swing.*;
import atlasdsl.*;
import atlassharedclasses.Point;
import faultgen.FaultGenerator;

public class GUITest {  
	
	private class RobotChoiceListener implements ActionListener {
		public void actionPerformed(ActionEvent e) {
			JComboBox cb = (JComboBox)e.getSource();
	        chosenRobotName = (String)cb.getSelectedItem();
		}
	}	
	
	private class FaultButtonListener implements ActionListener {
		public void actionPerformed(ActionEvent e) {
			// TODO: this time will not be respected yet until
			// time updates from MOOS are flowing through
			double testTimeLength = 20.0;
			// TODO: get the robot ID and time length from the GUI
			faultGen.injectSpeedFaultNow(testTimeLength, chosenRobotName);
			System.out.println("Injecting new fault from GUI");
		}
	}
	
	JFrame f;
    HashMap<Robot,JLabel> robotLabels = new LinkedHashMap<Robot, JLabel>();
    private Mission mission;
    private FaultButtonListener buttonListener = new FaultButtonListener();
    private RobotChoiceListener robotChoiceListener = new RobotChoiceListener();
    private FaultGenerator faultGen;
    private String chosenRobotName = "";
   
    private void setupLabels() {
    	int y = 0;
    	for (Robot r : mission.getAllRobots()) {
    		chosenRobotName = r.getName();
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
              
    	JButton injectButton=new JButton("Inject Overspeed Fault");//creating instance of JButton  
    	injectButton.setBounds(130,100,100, 40);
    	
    	List<String> robotNames = mission.getAllRobots().stream().map(r -> r.getName()).collect(Collectors.toList());
    	JComboBox robotChoice = new JComboBox(robotNames.toArray());
    	robotChoice.setSelectedIndex(1);
    	robotChoice.addActionListener(robotChoiceListener);
    	
    	// TODO: need a drop-down to select the robots to inject
        injectButton.addActionListener(buttonListener);

    	f.add(robotChoice);
    	f.add(injectButton);
    	
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
