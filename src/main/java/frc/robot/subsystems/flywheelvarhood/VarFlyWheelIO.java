package frc.robot.subsystems.flywheelvarhood;

public interface VarFlyWheelIO {

    /** Sets the voltage of the drivetrain motors (open loop) */
    public default void setVoltage(double leftVolts, double rightVolts) {
    }

    /** Sets the velocity of the drivetrain motors (closed loop) */
    public default void setVelocity(double flySpeed,
            double topSpeed, double flyFFValue, double topFFValue) {
    }

    /** Change the mode of the motors */
    public default void setBrakeMode(boolean enabled) {
    }

    /** Set PID constants */
    public default void setPID(double kP, double kI, double kD) {
    }

    public default void switchState(boolean extend) {

    }

    public default boolean getState() {
        return false;
    }

    public default void stop() {

    }

}