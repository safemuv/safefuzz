package middleware.gui;

import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.awt.image.MemoryImageSource;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javax.swing.*;
import atlasdsl.*;
import atlassharedclasses.Point;
import atlassharedclasses.Region;
import faultgen.FaultGenerator;

public class GUITest {  
	
	JFrame f;
    HashMap<Robot,JLabel> robotLabels = new LinkedHashMap<Robot, JLabel>();
    HashMap<Goal,JLabel> goalLabels = new LinkedHashMap<Goal, JLabel>();
    HashMap<Region,Image> regionImages = new LinkedHashMap<Region, Image>();
    
    
    private Mission mission;
    private FaultButtonListener buttonListener = new FaultButtonListener();
    private RobotChoiceListener robotChoiceListener = new RobotChoiceListener();
    private FaultGenerator faultGen;
    private String chosenRobotName = "";
	
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
	
    private void setupLabels() {
    	int y = 0;
    	for (Robot r : mission.getAllRobots()) {
    		chosenRobotName = r.getName();
	    	JLabel l = new JLabel(robotLabelText(r));
	    	l.setVisible(true);
	    	robotLabels.put(r, l);
	    	f.add(l);
    	}
    	
    	Map <String,Goal> goals = mission.getGoalsAndNames();
		for (Map.Entry<String,Goal> es : goals.entrySet()) {
			String name = es.getKey();
			Goal g = es.getValue();
			JLabel l = new JLabel(goalLabelText(g));
			l.setVisible(true);
			goalLabels.put(g, l);
			f.add(l);
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
    
    public void paint(Graphics g) {
    	Graphics2D g2d = (Graphics2D)g;
    	
    	for (Map.Entry<Region, Image> me :regionImages.entrySet()) {
    		Image img = me.getValue();
    		g2d.drawImage(img, 0, 0, null);
    	}
    }
    
    private void updateRegionImage(Region r, Image img) {
    	regionImages.replace(r, img);
    }
    
    private void renderPositionTracker(Region r, PositionTracker pt) {
    	int x = pt.getXSize();
    	int y = pt.getYSize();
    	int[][] counts = pt.getCounts();
    	
    	// Convert the 2D pixel array into 1D
    	int[] pixels = Arrays.stream(counts)
    	        .flatMapToInt(Arrays::stream)
    	        .toArray();
    	
    	MemoryImageSource ms = new MemoryImageSource(x,y,pixels,0,x);
    	Image img = Toolkit.getDefaultToolkit().createImage(ms);
    	System.out.println(img);
    	updateRegionImage(r, img);
    }
    
    private void renderCollectiveSensorCover(SensorCover csc) {
    	Map<Region,PositionTracker> pts = csc.getPosTrackers();
    	for (Map.Entry<Region,PositionTracker> me : pts.entrySet()) {
    		Region r = me.getKey();
    		PositionTracker pt = me.getValue();
    		renderPositionTracker(r,pt);
    	}
    }
    
    private void updateGoalInfo() {
    	// Render all the coverage goals visually - need to see the
    	for (Goal g : goalLabels.keySet()) {
    		GoalAction ga = g.getAction();
    		if (ga instanceof SensorCover) {
    			SensorCover csc = (SensorCover)ga;
    	   		// TODO: This call will likely be slow... may have to only do it
        		// periodically, e.g. 1/10th of calls 
    			renderCollectiveSensorCover(csc);
    		}
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
    	List<String> goalNames = mission.getAllGoalNames();	
    	
    	JComboBox robotChoice = new JComboBox(robotNames.toArray());
    	robotChoice.setSelectedIndex(1);
    	robotChoice.addActionListener(robotChoiceListener);
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
    			updateGoalLabels();
    			updateGoalInfo();
    			f.repaint();
			}
		});
    }
}
