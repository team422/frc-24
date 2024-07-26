package frc.lib.hardwareprofiler;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class SubsystemInterfacer {
    
    public Supplier<Double> currentPower;
    public Supplier<Double> currentVoltage;

    public Supplier<Double> currentTemperature;

    public Supplier<Double> currentVelocity;
    public Supplier<Double> currentPosition;

    public Consumer<Object> setPose;
    public Consumer<Object> setVelocity;
     
    public Supplier<Boolean> isAtSetpoint;



    public SubsystemInterfacer(Supplier<Double> currentPower, Supplier<Double> currentVoltage, Supplier<Double> currentTemperature, Supplier<Double> currentVelocity, Supplier<Double> currentPosition, Consumer<Object> setPose, Consumer<Object> setVelocity, Supplier<Boolean> isAtSetpoint) {
        this.currentPower = currentPower;
        this.currentVoltage = currentVoltage;
        this.currentTemperature = currentTemperature;        
        this.currentVelocity = currentVelocity;
        this.currentPosition = currentPosition;

        this.setPose = setPose;
        this.setVelocity = setVelocity;
        this.isAtSetpoint = isAtSetpoint;
    } 
}
