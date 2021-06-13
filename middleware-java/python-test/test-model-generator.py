#!/usr/bin/python3
import pyecore
from pyecore.resources import ResourceSet, URI, Ecore

rset = ResourceSet()
mm_resource = rset.get_resource(URI('/home/jharbin/academic/atlas/atlas-middleware/ecore-metamodels/com.github.atlas.dsl/fuzzspec.ecore'))
mm_root = mm_resource.contents[0]
rset.metamodel_registry[mm_root.nsURI] = mm_root

m = rset.get_resource(URI('/home/jharbin/academic/atlas/atlas-middleware/atlas-models/case-study-ros/fuzzing-example.model'))
model_root = m.contents[0]

m_new = rset.create_resource(URI('new-fuzzspec.model'));

MISSION_START_TIME = 0.0;
MISSION_END_TIME = 100.0;

RootClass = mm_root.getEClassifier("FuzzingSpecification");
SimClass = mm_root.getEClassifier("Simulator");
VSClass = mm_root.getEClassifier("VariableSpecification");
TimeLimitClass = mm_root.getEClassifier("TimingSpec");
ComponentClass = mm_root.getEClassifier("FuzzingComponent");
ROSVarTypeClass = mm_root.getEClassifier("ROSVariableType");
VarDir = mm_root.getEClassifier("VariableDirection");

def print_all_contents(element):
    contents = element.eContents
    for aname in dir(element):
        print(aname + "=" + str(element.eGet(aname)));
    for e in contents:
        print_all_contents(e);


def new_model():
    model_root = FuzzingSpecification();

def create_root_element():
    Rootclass = Ecore.EClass("FuzzingSpecification");
    myroot = Rootclass();
    return myroot

def create_component(sim, name, relPath):
    cm = ComponentClass();
    cm.name = name;
    cm.componentRelativePath = relPath;
    sim.components.append(cm);
    return cm;

def create_topic_entry(outBoundComponent, topic_name, reflect_name, rosTypeName, dir=1, vehicleSpecific = True, timeLimitFuzzStart = MISSION_START_TIME, timeLimitFuzzEnd = MISSION_END_TIME):
    vs = VSClass();
    vs.variable = topic_name;
    vs.reflectionName = reflect_name;
    tm = TimeLimitClass();
    rostype = ROSVarTypeClass();
    
    tm.startTime = timeLimitFuzzStart;
    tm.endTime = timeLimitFuzzEnd;
    rostype.ROSTypename = rosTypeName;
    vs.timeLimit = tm;
    vs.component = outBoundComponent;
    vs.vehicleSpecific = vehicleSpecific;
    vs.vtype = rostype;
    # Not sure how to address the direction enum properly.
    # For now, just hardcoded for OUTBOUND
    vs.dir = VarDir.eLiterals[dir];
    outBoundComponent.vars.append(vs);
    return vs;


def create_model_for_simulator(filename, simName, baseDirectory):
    newmodel = rset.create_resource(URI(filename));
    newmodel.use_uuid = True;
    fuzzspec = RootClass();
    newmodel.append(fuzzspec);
    sim = SimClass();
    sim.name = simName;
    fuzzspec.sims.append(sim);
    return newmodel, sim;

def main():
    fuzzmodel, sim = create_model_for_simulator("new-fuzzspec.model", "ROS", "/home/jharbin/catkin_ws/src/safemuv");
    ual = create_component(sim, "/ual",  "/ual");
    create_topic_entry(ual, "/ual/set_velocity_prime", "/ual/set_velocity", "geometry_msgs/TwistStamped");
    fuzzmodel.save();

main();
