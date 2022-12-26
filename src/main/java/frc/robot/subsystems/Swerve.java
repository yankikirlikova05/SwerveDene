package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

  //reference: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html

  SwerveModule[] modules = {
    new SwerveModule("FR", Constants.FR_angle, Constants.FR_power, 0, 0.0),
    new SwerveModule("FL", Constants.FL_angle, Constants.FL_power, 1, 0.0),
    new SwerveModule("RL", Constants.RL_angle, Constants.RL_power, 2, 0.0),
    new SwerveModule("RR", Constants.RR_angle, Constants.RR_power, 3, 0.0)
  };

  /*SwerveModuleState[] swerveModuleStates = {
    new SwerveModuleState(modules[0].getDriveDistance(),new Rotation2d(modules[0].getAngleDouble())),
    new SwerveModuleState(modules[1].getDriveDistance(),new Rotation2d(modules[1].getAngleDouble())),
    new SwerveModuleState(modules[2].getDriveDistance(),new Rotation2d(modules[2].getAngleDouble())),
    new SwerveModuleState(modules[3].getDriveDistance(),new Rotation2d(modules[3].getAngleDouble()))
  };*/

  int vx;
  int vy;
  int omega;

  //x, y, omega
  ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

  // Locations for the swerve drive modules relative to the robot center.
  //Edit the data 
  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );


  // Convert to module states
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

  SwerveModuleState frontLeft = moduleStates[0];
  SwerveModuleState frontRight = moduleStates[1];
  SwerveModuleState backLeft = moduleStates[2];
  SwerveModuleState backRight = moduleStates[3];


  
  


  public Swerve() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new Rotation2d(modules[1].getAngleDouble()));
    var frontRigtOptimized = SwerveModuleState.optimize(frontLeft,new Rotation2d(modules[0].getAngleDouble()));
    var rearLeftOptimized = SwerveModuleState.optimize(frontLeft, new Rotation2d(modules[2].getAngleDouble()));
    var rearRightOptimized = SwerveModuleState.optimize(frontLeft,new Rotation2d(modules[3].getAngleDouble()));


  }
}
