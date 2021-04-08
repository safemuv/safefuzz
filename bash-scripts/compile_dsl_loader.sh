#!/bin/sh
cd /home/atlas/atlas/atlas-middleware/middleware-java
javac -d /home/atlas/atlas/atlas-middleware/middleware-java/target/classes/ -classpath .:/home/atlas/atlas/atlas-middleware/middleware-java/target/classes/ src/atlasdsl/loader/GeneratedDSLLoader.java
