# Running the experiments

## Dependencies
- The Eclipse setup instructions
- Setup [XXXXXX_metrics Docker container](https://github.com/safemuv/safemuv_ros/blob/devel/safemuv_dockers/safemuv_metrics/README.md)
- Setup [metrics_test Docker container](https://github.com/safemuv/safemuv_ros/tree/aerolab-experiments/safemuv_dockers/safemuv_metrics/README.md)

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

To run the time-based fuzzing GA experiments and evolve solutions, go to the directory under the root
of the repository
**./expt-working/ros/**

and run:
```
./run_feedback_experiment_jmetal.sh
```

To run the condition-based fuzzing GA experiments and evolve solutions, go to the directory under the root
of the repository:
**./expt-working/ros/**

and run:
```
./run_feedback_experiment_jmetal_condition.sh
```
