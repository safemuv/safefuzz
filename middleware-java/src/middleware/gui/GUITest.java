package middleware.gui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.GridLayout;
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
	
	JFrame f;
	JPanel robotsPanel = new JPanel();
	JPanel goalsPanel = new JPanel();
	JPanel faultPanel = new JPanel();
	
    HashMap<Robot,JLabel> robotLabels = new LinkedHashMap<Robot, JLabel>();
    HashMap<Goal,JLabel> goalLabels = new LinkedHashMap<Goal, JLabel>();
    
    private Mission mission;
    private FaultButtonListener buttonListener = new FaultButtonListener();
    private RobotChoiceListener robotChoiceListener = new RobotChoiceListener();
    private FaultGenerator faultGen;
    private String chosenRobotName = "";
    
    private PositionTrackingOutput ptPanel;
	
	private class RobotChoiceListener implements ActionListener {
		public void actionPerformed(ActionEvent e) {
			JComboBox<?> cb = (JComboBox<?>)e.getSource();
	        chosenRobotName = (String)cb.getSelectedItem();
		}
	}
	
	public HashMap<Goal,JLabel> getGoalLabels() {
		return goalLabels;
	}
	
	private class FaultButtonListener implements ActionListener {
		public void actionPerformed(ActionEvent e) {
			// TODO: get the time length from the GUI
			double testTimeLength = 8.0;
			faultGen.injectSpeedFaultNow(testTimeLength, chosenRobotName);
			System.out.println("Injecting new fault from GUI");
		}
	}
	
    private void setupLabels() {
    	for (Robot r : mission.getAllRobots()) {
    		chosenRobotName = r.getName();
	    	JLabel l = new JLabel(robotLabelText(r));
	    	l.setVisible(true);
	    	robotLabels.put(r, l);
	    	robotsPanel.add(l);
    	}
    	
    	Map <String,Goal> goals = mission.getGoalsAndNames();
		for (Map.Entry<String,Goal> es : goals.entrySet()) {
			//String name = es.getKey();
			Goal g = es.getValue();
			JLabel l = new JLabel(goalLabelText(g));
			l.setVisible(true);
			goalLabels.put(g, l);
			goalsPanel.add(l);
		}
    }
    
    private String robotLabelText(Robot r) {
    	try {
    		Point loc = r.getPointComponentProperty("location");
    		return r.getName() +  " @ " + loc.toString();
    	} catch (MissingProperty e) {
    		return r.getName() + "<missing location>";
    	}
    }
    
    private String goalLabelText(Goal g) {
    	return g.getName() + " - " + g.getStatus().toString();
    }
    
    private void updateLabels() {
    	for (Map.Entry<Robot, JLabel> entry : robotLabels.entrySet()) {
    		JLabel l = entry.getValue();
    		Robot r = entry.getKey();
    		String text = robotLabelText(r);
    		l.setText(text);
    		l.repaint();
    	}
    }
    
    private void updateGoalLabels() {
    	for (Map.Entry<Goal,JLabel> entry : goalLabels.entrySet()) {
    		JLabel l = entry.getValue();
    		Goal g = entry.getKey();
    		l.setText(goalLabelText(g));
    	}
    }
    
    public void paintComponent(Graphics g) {
    	
    }
    
    public GUITest(Mission mission, FaultGenerator faultGen) {
    	this.mission = mission;
    	this.faultGen = faultGen;
    	
    	f=new JFrame();
    	
    	ptPanel = new PositionTrackingOutput(this);
    	ptPanel.setBackground(new Color(0,0,0));
    	ptPanel.setSize(100,100);
    	ptPanel.setVisible(true);
    	
    	f.setLayout(new GridLayout(2,2));
    	setupLabels();
    	
    	f.add(robotsPanel);
    	f.add(goalsPanel);
    	f.add(faultPanel);
              
    	JButton injectButton=new JButton("Inject Overspeed Fault");//creating instance of JButton  
    	JTextField faultLen = new JTextField("Fault Length");
    	injectButton.setBounds(130,100,100, 40);

    	
    	List<String> robotNames = mission.getAllRobots().stream().map(r -> r.getName()).collect(Collectors.toList());
    	
    	JComboBox<?> robotChoice = new JComboBox(robotNames.toArray());
    	robotChoice.setSelectedIndex(1);
    	robotChoice.addActionListener(robotChoiceListener);
        injectButton.addActionListener(buttonListener);

        faultPanel.add(robotChoice);
    	faultPanel.add(injectButton);
    	faultPanel.add(faultLen);
    	//f.getContentPane().add(ptPanel, BorderLayout.CENTER);
    	f.getContentPane().add(ptPanel);
    	
    	f.setSize(500,500);
    	f.setVisible(true);
    	
    	f.repaint();
    	ptPanel.repaint();
    }
    
    public synchronized void updateGUI() {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
    			updateLabels();
    			updateGoalLabels();
    			ptPanel.updateGoalInfo();
    			
    			f.repaint();
    			ptPanel.repaint();
			}
		});
    }
}
