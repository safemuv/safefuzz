# Running the experiments - using the fully integrated SAFEMUV system

## Dependencies
- The Eclipse setup instructions
- Setup [safemuv_metrics Docker container](https://github.com/safemuv/safemuv_ros/blob/devel/safemuv_dockers/safemuv_metrics/README.md)
- Setup [metrics_test Docker container](https://github.com/safemuv/safemuv_ros/tree/aerolab-experiments/safemuv_dockers/safemuv_metrics/README.md)

## Install the SAFEMUV knowledge graph components
Using the information in [knowledge_graph](https://github.com/safemuv/knowledge_graph)
- Install Neo4j and Neo4j destkop
- Add new database safemuv with password safemuv 
- Setup python virtual environment
- Setup the python knowledge graph components
- Run the instructions under "Populate the graph" to set up the graph

After doing this, set up the virtual environment path for Python as the variable VENV_PATH in 
**/home/safemuv/academic/atlas/atlas-middleware/expt-working/ros/generate_launch_files_rmkg.sh**

## Install the Scenario Generation Components

As shown in [scenario_generation](https://github.com/safemuv/scenario_generation),
add the scenario generation components to the catkin workspace:

```
cd catkin_ws/src
git clone https://github.com/safemuv/scenario_generation.git
catkin build
```
These components will be called to generate the modified scenario launch and config files,
which will be stored, including the configuration files in
**/home/safemuv/catkin_ws/src/scenario_generation/safemuv_scenarios/scenarios**

## Installing the Model Generator

Install the [model generator](https://github.com/safemuv/integration_tools/tree/master/graph-to-model-conversion)
and enter the same Python virtualenv set up previously in VENV_PATH.

```
./fuzzing_model_generator.py
./mission_model_generator.py
```

TODO: check the syntax for these calls
Copy the generated files into the atlas-models project directory

## How to run the experiments

Before running the experiments, go to the scripts directory of this
instructions repository and run the following script (it may ask for the root password):
```
./patch_protocol.sh 
```

This will activate a change for Python libraries that is necessary for
rosbridge to work.

After running them, use in the same directory in order to restore them:
```
./restore_protocol.sh 
```

To run the middleware to connect to a live version of the experiment,
activate the "ROS-Middleware" run configuration when ROS live
experiments are started.

To run the time-based fuzzing GA experiments and evolve a solution, go to the directory
**~/academic/atlas/atlas-middleware/expt-working/ros/**

and run:
```
./run_feedback_experiment_jmetal.sh
```

To run the condition-based fuzzing GA experiments and evolve a solution, go to the directory
**~/academic/atlas/atlas-middleware/expt-working/ros/**

and run:
```
./run_feedback_experiment_jmetal_condition.sh
```
