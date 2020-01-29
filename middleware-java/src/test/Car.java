package test;

public class Car {
    private String brand = null;
    private int doors = 0;

    public Car() {
    	super();
    }
    
    public Car(String brand, int doors) {
    	this.doors = doors;
    	this.brand = brand;    	
    }
    
    public String getBrand() { return this.brand; }
    public void   setBrand(String brand){ this.brand = brand;}

    public int  getDoors() { return this.doors; }
    public void setDoors (int doors) { this.doors = doors; }
}