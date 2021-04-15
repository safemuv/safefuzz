package test.reloadclass;

import java.io.IOException;
import java.net.URL;
import java.net.URLClassLoader;

public class ReloadClass
{
  public static void main(String[] args)
  throws ClassNotFoundException, IOException {
    Class<?> myClass=ReloadClass.class;
    System.out.printf("my class is Class@%x%n", myClass.hashCode());
    System.out.println("reloading");
    URL[] urls={ myClass.getProtectionDomain().getCodeSource().getLocation() };
    ClassLoader delegateParent = myClass.getClassLoader().getParent();
    try(URLClassLoader cl=new URLClassLoader(urls, delegateParent)) {
      Class<?> reloaded=cl.loadClass(myClass.getName());
      System.out.printf("reloaded my class: Class@%x%n", reloaded.hashCode());
      System.out.println("Different classes: "+(myClass!=reloaded));
    }
  }
}