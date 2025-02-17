package frc.lib.util;


public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double speedAdjustmentFactor;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double speedAdjustmentFactor) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.speedAdjustmentFactor = speedAdjustmentFactor;
    }
}
