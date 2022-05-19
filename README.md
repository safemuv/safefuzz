# SAFEFuzz Infrastructure
## Introduction

<img src="safefuzz2.png" alt="SAFEFuzz Architecture" width="50%" height="50%" />

SAFEFuzz provides a model-based testing approach (incorporating fuzz
testing) that supports the identification of mission safety
requirement violations in robotic systems.

SAFEFuzz employs a domain-specific language for the mission and
fuzzing specification, thus raising the level of abstraction by
abstracting away low-level simulator-specific details and providing
simulator independence.

To this end, SAFEFuzz enables roboticists and test experts to easily
specify the fuzzing space and launch two distinct evolutionary-guided
fuzzing synthesis techniques:

* a time-based variant focused on timing intervals where deviations
  from safety requirements can occur

* a condition-based variant that utilises a grammar to yield the
  events leading to these deviations.

To achieve these, SAFEFuzz leverages the publish-subscribe
protocol underpinning widely-used robotic simulators (e.g., ROS/Gazebo
and MOOS-IvP and is underpinned by an extensible middleware that
supports separation of concerns between fuzzing and the target robotic
simulator.

To the best of our knowledge, SAFEFuzz is the first model-driven,
systematic, and automated testing approach that employs fuzzing to
establish the dependability levels of a robotic system under test
while providing simulation independence and the permitting test
engineers to implement custom scenario-specific fuzzing operations.

## Repository Structure

TODO: describe repository structure

## Results
TODO: add results from the remaining experiments here
