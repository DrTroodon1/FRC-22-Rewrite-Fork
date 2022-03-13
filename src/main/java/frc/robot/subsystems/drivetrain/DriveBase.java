package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
    // Static final for now, make sure to change in order to switch between drives
    private static final double wheelRadiusMeters = 0.0508;
    private static final double maxVelocityMetersPerSec = Units.feetToMeters(16.5);
    private final double kS;
    private final double kV;
    private final double kA;
    private final DriveIO driveIO;

    private final SimpleMotorFeedforward leftFF;
    private final SimpleMotorFeedforward rightFF;

    public DriveBase(DriveIO driveIO) {
        this.driveIO = driveIO;
        this.kS = 0.66;
        this.kV = 0.0503;
        this.kA = 0.00516;
        // Arbitrary values for now
        leftFF = new SimpleMotorFeedforward(kS,kV,kA);
        rightFF = new SimpleMotorFeedforward(kS,kV,kA);
    }

    public void driveVoltage(double leftVoltage, double rightVoltage) {
        driveIO.setVoltage(leftVoltage, rightVoltage);
    }

    public void drivePercent(double leftPercent, double rightPercent) {
        driveVelocity(leftPercent * maxVelocityMetersPerSec,
            rightPercent * maxVelocityMetersPerSec);
    }

    public void driveVelocity(double leftVelocity, double rightVelocity) {
        double leftVelocityRadPerSec = leftVelocity / wheelRadiusMeters;
        double rightVelocityRadPerSec = rightVelocity / wheelRadiusMeters;
        double leftFFValue = leftFF.calculate(leftVelocityRadPerSec);
        double rightFFValue = rightFF.calculate(rightVelocityRadPerSec);

        driveIO.setVelocity(leftVelocityRadPerSec, rightVelocityRadPerSec, leftFFValue, rightFFValue);
    }

    public void setBrakeMode(boolean enable) {
        driveIO.setBrakeMode(enable);
    }
}