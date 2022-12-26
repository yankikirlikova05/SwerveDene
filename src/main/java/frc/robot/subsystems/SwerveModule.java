// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.lang.Math;

public class SwerveModule{
  WPI_TalonFX angle_motor;
  WPI_TalonFX drive_motor;

  DutyCycleEncoder srx_Encoder;

  double module_offset;

  String module_name;

  //Angle PIDs
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;

  //Drive PIDs
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;

  PIDController angle_pid = new PIDController(p, i, d);

  PIDController drive_pid = new PIDController(kp, ki, kd);

  //EDIT
  double wheel_radius = 0.0;

  public SwerveModule(String moduleName, int anglePort, int drivePort, int encoderPort, double module_offset) {
    this.module_name = moduleName;
    angle_motor = new WPI_TalonFX(anglePort);
    drive_motor = new WPI_TalonFX(drivePort);
    srx_Encoder = new DutyCycleEncoder(encoderPort);
    this.module_offset = module_offset;
    srx_Encoder.setPositionOffset(module_offset);
  }

  public double getAngleDouble(){
    return srx_Encoder.getAbsolutePosition();
  }

  public double anglePID(double desired_angle){
    return angle_pid.calculate(getAngleDouble(), desired_angle);
  } 

  public double getDriveDistance(){
    return 2*Math.PI*(drive_motor.getSelectedSensorPosition()/2048)*wheel_radius;
  }

  public double drivePID(double desired_drive){
    return drive_pid.calculate(2*Math.PI*(drive_motor.getSelectedSensorPosition()/2048)*wheel_radius,desired_drive);
  }

  public void zeroTheWheel(){
    anglePID(0.0);
  }


}
