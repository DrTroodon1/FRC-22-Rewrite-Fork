package frc.robot.subsystems.intake;

public interface IntakeIO {
    
    public default void runIntakePercent(double percent){}

    public default void runIntakeVoltage(double volts){}
    
    public default void runIntakeVelocity(double velocity){}

    public default void getIntakeVelocity(){}

    public default void stopIntake(){}

    public default void setIntakeMotorBrakeMode(boolean enable){}

    public default void setIntakeSolenoid(boolean extended){}

    public default boolean getState() {
        return false;
    }


}
