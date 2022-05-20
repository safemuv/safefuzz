# Troubleshooting - potential problems

## If you get an exception "java.lang.UnsupportedClassVersionError"
Exception in thread "main" java.lang.UnsupportedClassVersionError: it/units/malelab/jgea/core/operator/Mutation has been compiled by a more recent version of the Java Runtime (class file version 58.0), this version of the Java Runtime only recognizes class file versions up to 55.0
```
        at java.base/java.lang.ClassLoader.defineClass1(Native Method)
        at java.base/java.lang.ClassLoader.defineClass(ClassLoader.java:1017)
        at java.base/java.security.SecureClassLoader.defineClass(SecureClassLoader.java:174)
        at java.base/jdk.internal.loader.BuiltinClassLoader.defineClass(BuiltinClassLoader.java:800)
        at java.base/jdk.internal.loader.BuiltinClassLoader.findClassOnClassPathOrNull(BuiltinClassLoader.java:698)
        at java.base/jdk.internal.loader.BuiltinClassLoader.loadClassOrNull(BuiltinClassLoader.java:621)
        at java.base/jdk.internal.loader.BuiltinClassLoader.loadClass(BuiltinClassLoader.java:579)
        at java.base/jdk.internal.loader.ClassLoaders$AppClassLoader.loadClass(ClassLoaders.java:178)
        at java.base/java.lang.ClassLoader.loadClass(ClassLoader.java:522)
        at java.base/java.lang.ClassLoader.defineClass1(Native Method)
        at java.base/java.lang.ClassLoader.defineClass(ClassLoader.java:1017)
        at java.base/java.security.SecureClassLoader.defineClass(SecureClassLoader.java:174)
        at java.base/jdk.internal.loader.BuiltinClassLoader.defineClass(BuiltinClassLoader.java:800)
        at java.base/jdk.internal.loader.BuiltinClassLoader.findClassOnClassPathOrNull(BuiltinClassLoader.java:698)
        at java.base/jdk.internal.loader.BuiltinClassLoader.loadClassOrNull(BuiltinClassLoader.java:621)
        at java.base/jdk.internal.loader.BuiltinClassLoader.loadClass(BuiltinClassLoader.java:579)
        at java.base/jdk.internal.loader.ClassLoaders$AppClassLoader.loadClass(ClassLoaders.java:178)
        at java.base/java.lang.ClassLoader.loadClass(ClassLoader.java:522)
        at fuzzexperiment.runner.jmetal.FuzzingSelectionsMutation.<init>(FuzzingSelectionsMutation.java:95)
        at fuzzexperiment.runner.jmetal.JMetalExpt.jMetalRun(JMetalExpt.java:157)
        at fuzzexperiment.runner.jmetal.main.RunJMetal_timebased.main(RunJMetal_timebased.java:36)
```

The fix is to select project "JGEA", right-click, select "Java
Compiler" and change all the compiler compliance level, generated
class files compatibiltiy and the source compatibility to the same as
the com.github.safemuv.platform project (on my test system, 11)

# If there are problems with the build path (after setting up projects)
Right click on the project "JGEA",
select "Build path" ... "configure build path". Select "Libraries"
and remove "JRE System Library". Then click "Classpath" and "Add
Library".

Then choose "JRE System Library"... "Workspace default JRE (...)"
Then select "Finish" and "Apply and Close"
Hopefully errors will be resolved.
You may have to select "Project"/"Clean" to resolve errors
        
# Problems with empty calibration points
        
If there are errors about "nan" values in the transforms, check the calibration
points are not empty. This may be caused by not sourcing the catkin
workspace before populating the graph.
        
