# PDDL4J library

## 1. Description

This is an implementation of a Monte Carlo Tree Search planner which uses Pure Random Walks as a planning procedure. It was implemented in Java with the help of PDDL4J.

## 2. Dependencies

  * [Java JDK](https://adoptopenjdk.net/>) version 8 or higher.
  * [Gradle](https://gradle.org/>) to build the library.

## 3. How to use the PDDL4J library?

A complete documentation of the library is [available online](http://pddl4j.imag.fr/). For those in a hurry to get
PDDL4J and start running a planner start reading the [Getting Started section](http://pddl4j.imag.fr/getting_started.html).
Note that [PDDL4J API documentation](http://pddl4j.imag.fr/api_documentation.html) is also available online.

to compile the code use
'''
javac -d classes -cp build/libs/pddl4j-4.0.0.jar src/main/java/fr/uga/pddl4j/montecarlo/*.java
'''

to run the code use 
'''
java -cp classes:build/libs/pddl4j-4.0.0.jar fr.uga.pddl4j.montecarlo.MonteCarloTreeSearch
'''

The planner was tested against HSP on benchmarks blocks, gripper, logistics and depots. However depots are computationally expensive hence they have been ommited.
