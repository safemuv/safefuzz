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
import java.util.Optional;
import java.util.stream.Collectors;

import javax.swing.*;
import atlasdsl.*;
import atlasdsl.faults.Fault;
import atlassharedclasses.Point;

import faultgen.FaultGenerator;
import middleware.core.ATLASCore;

public class GUITest {
	
	private final double DEFAULT_FAULT_TIME_LENGTH = 10.0;
	
	JLabel timeLabel = new JLabel("time = <...>");
	
	JFrame f;
	JPanel robotsPanel = new JPanel();
	JPanel goalsPanel = new JPanel();
	JPanel faultPanel = new JPanel();
	JPanel fuzzingPanel = new JPanel();
	
	JTextField faultLen;
	
    HashMap<Robot,JLabel> robotLabels = new LinkedHashMap<Robot, JLabel>();
    HashMap<Goal,JLabel> goalLabels = new LinkedHashMap<Goal, JLabel>();
    HashMap<String,JLabel> fuzzingKeyLabels = new LinkedHashMap<String, JLabel>();
    
    // This shows the active fuzzing key operations at any time during execution
    HashMap<String,String> fuzzingKeyOperations= new LinkedHashMap<String,String>();
    
    private Mission mission;
    private FaultButtonListener buttonListener = new FaultButtonListener();
    private FaultChoiceListener faultChoiceListener = new FaultChoiceListener();
    
    private ATLASCore core;
    
    private static GUITest guiRef;
    
    private FaultGenerator faultGen;
    private String chosenFault = "";
    
    private PositionTrackingOutput ptPanel;
	private String faultDefFile;

	private String fuzzingDefFile;
	
	
	public static GUITest getGUI() {
		return guiRef;
	}
	
	public static void setGUI(GUITest gui) {
		if (guiRef == null) {
			guiRef = gui;
		}
	}
	
	private class FaultChoiceListener implements ActionListener {
		public void actionPerformed(ActionEvent e) {
			JComboBox<?> cb = (JComboBox<?>)e.getSource();
	        chosenFault = (String)cb.getSelectedItem();
		}
	}
	
	public HashMap<Goal,JLabel> getGoalLabels() {
		return goalLabels;
	}
	
	private class FaultButtonListener implements ActionListener {
		
		public void actionPerformed(ActionEvent e) {
			double faultTimeLength = DEFAULT_FAULT_TIME_LENGTH;
			
			try {
				String faultTimeLength_s = faultLen.getText();
				faultTimeLength = Double.valueOf(faultTimeLength_s);
			} catch (NumberFormatException ex) {
				System.out.println("Fault time length not set - assuming default");
			}
 
			Optional<Fault> f_o = mission.lookupFaultByName(chosenFault);
			if (f_o.isPresent()) {
				Fault f = f_o.get();
				faultGen.injectDynamicFault(f, faultTimeLength, Optional.empty());
				System.out.println("Injecting new fault from GUI");
			} else {
				System.out.println("Could not find fault " + chosenFault + " in model");
			}
		}
	}
	
    private void setupLabels() {
    	timeLabel.setVisible(true);
    	robotsPanel.add(timeLabel);
    	
    	for (Robot r : mission.getAllRobots()) {
    		//chosenRobotName = r.getName();
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
		
		for (Map.Entry<String,String> es : fuzzingKeyOperations.entrySet()) {
			JLabel l = new JLabel();
			l.setVisible(true);
			String k = es.getKey();
			fuzzingKeyLabels.put(k,l);
			
		}
    }
    
    private String robotLabelText(Robot r) {
    	try {
    		Point loc = r.getPointComponentProperty("location");
    		return r.getName() +  " @ " + loc.toString2DP();
    	} catch (MissingProperty e) {
    		return r.getName() + "<missing location>";
    	}
    }
    
    private String goalLabelText(Goal g) {
    	return g.getName() + " - " + g.getStatus().toString();
    }
    
    private void updateLabels() {
    	timeLabel.setText("time = " + Double.toString(core.getTime()));
    	
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
    
    private void updateFuzzingInfo() {
    	for (Map.Entry<String,JLabel> entry : fuzzingKeyLabels.entrySet()) {
    		JLabel l = entry.getValue();
    		String key = entry.getKey();
    		String text = fuzzingLabelText(key);
    		l.setText(text);
    		l.repaint();
    	}
    }
    
    private String fuzzingLabelText(String key) {
    	return key + "-" + fuzzingKeyOperations.get(key);
	}

	public void paintComponent(Graphics g) {
    	
    }
    
    @SuppressWarnings({ "rawtypes", "unchecked" })
	public GUITest(ATLASCore core, Mission mission, FaultGenerator faultGen) {
    	this.core = core;
    	this.mission = mission;
    	this.faultGen = faultGen;
    	setGUI(this);
    	
    	f=new JFrame();
    	f.setTitle("ATLAS Middleware - no faults defined");
    	
    	ptPanel = new PositionTrackingOutput(this);
    	ptPanel.setBackground(new Color(0,0,0));
    	ptPanel.setSize(100,100);
    	ptPanel.setVisible(true);
    	
    	f.setLayout(new GridLayout(2,2));
    	setupLabels();
    	
    	f.add(robotsPanel);
    	f.add(goalsPanel);
    	f.add(faultPanel);
    	f.add(fuzzingPanel);
              
    	JButton injectButton=new JButton("Inject Fault");
    	
    	faultLen = new JTextField("Fault Length");
    	injectButton.setBounds(130,100,100, 40);

    	List<String> robotNames = mission.getAllRobots().stream().map(r -> r.getName()).collect(Collectors.toList());
    	List<String> faultNames = mission.getFaultsAsList().stream().map(f -> f.getName()).collect(Collectors.toList());
    	
		JComboBox<?> robotChoice = new JComboBox(robotNames.toArray());
		JComboBox<?> faultChoice = new JComboBox(faultNames.toArray());
    	
    	robotChoice.setSelectedIndex(1);
        injectButton.addActionListener(buttonListener);
        faultChoice.addActionListener(faultChoiceListener);

        faultPanel.add(faultChoice);
    	faultPanel.add(injectButton);
    	faultPanel.add(faultLen);
    	
    	fuzzingPanel.setVisible(true);
    	
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
    			updateFuzzingInfo();
    			ptPanel.updateGoalInfo();
    			
    			f.repaint();
    			ptPanel.repaint();
			}
		});
    }
    
    public synchronized void setFuzzingKeyState(String key, String vehicle, String opText) {
    	String k = key + "-" + vehicle;
    	fuzzingKeyOperations.put(k, opText);
    	if (!fuzzingKeyLabels.containsKey(k)) {
    		JLabel l = new JLabel();
    		fuzzingKeyLabels.put(k, l);
    		fuzzingPanel.add(l);
    	}
    }
    
    // TODO: GUI should handle multiple fuzzing key operations here
    public synchronized void addFuzzingKeyState(String key, String vehicle, String opText) {
    	String k = key + "-" + vehicle;
    	fuzzingKeyOperations.put(k, "Possible multiple: opText");
    	if (!fuzzingKeyLabels.containsKey(k)) {
    		JLabel l = new JLabel();
    		fuzzingKeyLabels.put(k, l);
    		fuzzingPanel.add(l);
    	}
    }

	public void setFaultDefinitionFile(String filePath) {
		this.faultDefFile = filePath;
		f.setTitle("ATLAS Middleware - FAULTS DEFINED in " + faultDefFile);
	}

	public void setFuzzingDefinitionFile(String filename) {
		this.fuzzingDefFile = filename;
		f.setTitle("ATLAS Middleware - FUZZING FILE " + fuzzingDefFile);
	}
}
