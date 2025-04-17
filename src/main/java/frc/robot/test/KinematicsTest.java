package frc.robot.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class KinematicsTest {

    private static final Translation2d m_frontLeftLocation = Constants.SwerveConstants.m_frontLeftLocation;
    private static final Translation2d m_frontRightLocation = Constants.SwerveConstants.m_frontRightLocation;
    private static final Translation2d m_backLeftLocation = Constants.SwerveConstants.m_backLeftLocation;
    private static final Translation2d m_backRightLocation = Constants.SwerveConstants.m_backRightLocation;

    public static void main(String[] args) {
        SwerveDriveKinematics m_kinematics =
            new SwerveDriveKinematics(m_backLeftLocation, m_backRightLocation, m_frontRightLocation, m_frontLeftLocation);
        
        var modules = new SwerveModuleState[4];
        modules[0] = new SwerveModuleState(1, new Rotation2d(Math.PI/4.0));
        modules[1] = new SwerveModuleState(1, new Rotation2d(3*Math.PI/4.0));
        modules[2] = new SwerveModuleState(-1, new Rotation2d(Math.PI/4.0));
        modules[3] = new SwerveModuleState(1, new Rotation2d(-Math.PI/4.0));
        var s = m_kinematics.toChassisSpeeds(modules);
        System.out.println(s);
    }

}
