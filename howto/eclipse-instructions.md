# Middleware and Installation Instructions

## Prerequisties

This assumes:

- "USERNAME" as your username - please change throughout if it is different
- All the ROS and simulator components are installed
- Ubuntu installation (Ubuntu Desktop)

## Install Java JRE and ROSbridge components
At a terminal:

```
sudo apt-get install openjdk-11-jre-headless maven activemq
sudo apt-get install ros-melodic-rosbridge-server
```

## Installing middleware components
```
mkdir -p ~/academic/atlas
cd ~/academic/atlas
git clone --single-branch --branch integration-final https://github.com/jrharbin-york/atlas-middleware.git
```

JGEA is also a components which is installed (here I put it under a
source directory, but you can change this if you want to put it
elsewhere, you just have to update the location when setting up
Eclipse).

```
mkdir -p ~/source
cd ~/source
git clone https://github.com/jrharbin-york/jgea.git
```

## Eclipse setup

It is first necessary to install Eclipse, since it provides support
for the modelling tools used for the platform and mission
configuration.

If the Eclipse version in your repositories is sufficiently recent,
you can install the repository Eclipse and then go on to "Import the
core XXXXXX project". Otherwise, follow the steps below:

- Download Eclipse installer online (Tested with version 2021-06)
- Install Eclipse via the installer (2021-06)
- Select "Eclipse IDE for Java Developers"
- Install to selected path: /home/$USER/eclipse/java-2021-06
- Accept License

## Setup workspace
Setup workspace at ~/eclipse-workspace

## Add lines to .bashrc
Add to .bashrc
```
export PATH=/home/USERNAME/eclipse/java-2021-06/eclipse:$PATH
```

It can now be started via "eclipse" from the command prompt

## Import the core USERNAME project
- Select "Import projects" from the Package Explorer
- Select Maven / Existing Maven Projects
- Select browser and choose **/home/USERNAME/academic/atlas/atlas-middleware/middleware-java**,
which contains pom.xml

## Import the JGEA project (required for some of the GA experiments)
- Select "Import projects" from the Package Explorer
- Select Maven / Existing Maven Projects
- Select root directory: browse to "~/source/jgea"
- Import the JGEA project by clicking "Finish"

## Set JGEA classpath entry
- Right click "com.github.USERNAME.platform" and "Build Path"/"Configure Build Path"
- Select "Projects" tab and remove jgea
- Select "Add" and add "JGEA"

If there are problems with the build path after setting up the project, see the troubleshooting file.

## Install all the Eclipse Modelling Framework plugins
- Start Eclipse
- Select "Help" / "Install New Software"
- Select the update link for your Eclipse version e.g. "Work With" - "2021-06 - http://download.eclipse.org/releases/2021-06"
- Select "Modeling" and check all beneath it in the check box
  Select "Next" twice
  Accept license then "Finish"
  Wait for install to complete (indicated on "Installing Software" on lower right status bar) - it may freeze at 49% for a while
  Select "Restart" to apply updates

## Install Epsilon
- Select "Help" / "Install New Software"
- Under "Work with" enter "http://download.eclipse.org/epsilon/updates/2.3/" then click Add
- Under the check boxes select "Eclipse Core" and "Epsilon EMF Integration" and all beneath them
- Click "Next" twice
- Accept license then "Finish"
- Select "Restart" to apply updates

## Install Emfatic
- Select "Help" / "Install New Software"
- Under "Work with" enter "http://download.eclipse.org/emfatic/update" then click Add
- Under the check boxes select "Emfatic (Incubation)"
- Click "Next" twice
- Accept license then "Finish"
- Select "Continue with installation anyway" when asked
- Select "Restart" to apply updates

## Import the metamodel project
- Select "File, Import" and select "Projects from Folder or Archive"
- Select as Import Source, "Directory" and enter "/home/USERNAME/academic/atlas/ecore-metamodels"
- Select only "ecore-metamodels/com.github.atlas.dsl" in checkbox (second checkbox, deselect the first)
- Select Finish

## Generate/Register Ecore metamodels
- Open newly created project "com.github.atlas.dsl"
- Right click on "atlas.emf" and select "Generate Ecore model"
- Right click on "atlas.ecore" and select "Register Epackages"
- Right click on "fuzzspec.emf" and select "Generate Ecore model"
- Right click on "fuzzspec.ecore" and select "Register Epackages"

## Import the models and EGL code
- Select "File, Import" and select "Projects from Folder or Archive"
- Select as Import Source, "Directory" and enter "/home/USERNAME/academic/atlas/atlas-middleware/atlas-models"
- Select "Finish"
- This will import a new project entitled "models-and-egl"

## Create Run configurations

- Select Run Configurations, Java Application -> "New Launch configuration" icon
- Name it "ROS-Middleware"
- Set Main class "test.middleware.ROSLauncher"
- Set Arguments tab, under "Program arguments" enter "nofault gui"

- Select Run Configurations, EGL Generator -> "New launch configuration" icon
- Name it "GenMission"
- Select Browse Workspace and select "genmission.egl" (/models-and-egl/genmission.egl)
- Under "Text generated should be printed to" select "The following file" and browse to
- select GeneratedDSLLoader.java (full path "/com.github.USERNAME.platform/src/atlasdsl/loader/GeneratedDSLLoader.java")
- On the "Models" Tab, select "Add" and select "EMF Model"
- Enter "Name as Mission" and "Aliases" as "M"
- For "Model file" select "Browse Workspace" and find "models-and-egl/case-study-ros/mission.model"
- For "Metamodels" select "http://www.github.com/jrharbin-york/atlas-middleware/dsl/atlas" (if not automatically selected)
- "Run" should produce several lines in blue text in the console (and no errors)

- Select Run Configurations, EGL Generator -> "New launch configuration" icon
- Name it "GenFuzzSpec"
- Select Browse Workspace and select "genFuzzingEngine.egl" (/models-and-egl/genFuzzingEngine.egl)
- Under "Text generated should be printed to" select "The following file" and browse to
- select GeneratedFuzzingSpec.java (full path "/com.github.USERNAME.platform/src/fuzzingengine/spec/GeneratedFuzzingSpec.java")
- On the "Models" Tab, select "Add" and select "EMF Model"
- Enter Name as "FuzzSpec" and "Aliases" as "F"
- For "Model file" select "Browse Workspace" and find "models-and-egl/case-study-ros/fuzzing-example.model"
- For "Metamodels" select "http://www.github.com/jrharbin-york/atlas-middleware/dsl/fuzzspec" (if not automatically selected)
- Go to tab "Parameters":
- Add Parameter with name "simName" and value "ROS"
- Add Parameter with name "fuzzFileName" and value "/home/USERNAME/academic/atlas/atlas-middleware/fuzz-configs/ros-fuzztest.csv"
- Try "Run" for the run configuration and output should be generated successfully - no errors

# Setting up paths for the experiment runners
## Copy the execution path for the JVM/libs into java_cmd_string

In order for the automated runner scripts to run the middleware properly, the
JVM command line and  library flags need to be set up in the file
**/home/USERNAME/academic/atlas/atlas-middleware/expt-working/ros/java_cmd_string**.
Since this is potentially different for different machines, library versions and 
usernames, the best way is to obtain the necessary config from your Eclipse.

Open Eclipse and open the Run Configurations, select the
"ROS-Middleware" run configuration, and select "Show Command Line".

Copy the entire contents of the enclosed text box, starting with "/usr/lib/... java" 
excluding only the class name "test.middleware.ROSLauncher". The last element should
be "snakeyaml-1.26.jar"

Copy this path fragment into the file 
**/home/USERNAME/academic/atlas/atlas-middleware/expt-working/ros/java_cmd_string**
It should be all as one line.

## Check over the paths in the fuzzingexpt.config paths correctly

In the file,
**/home/USERNAME/academic/atlas/atlas-middleware/expt-working/ros/fuzzingexpt.config**
the usernames need to be set correctly. Open this file and ensure that
the usernames used on all 4 lines of paths matches your username.

## Set the correct "Base Directory" for ROS under the fuzzing model

In a terminal, go to the directory **~/academic/atlas/atlas-middleware/atlas-models**
and run the command:
```
./fix_model_path.sh USERNAME
```
where USERNAME is your username. This will give you the username set up properly in the model 
files (it is required to activate the model formulations)

# How to run the experiments
See [here](running-the-experiments.md)

# Troubleshooting
Known errors which have been encountered are documented [here](troubleshooting.md)
