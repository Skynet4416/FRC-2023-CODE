// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;


public final class Autos {
  /** Example static factory for an autonomous command. */
  private AHRS AHRSsensor;
  private double previousTime = 0;
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  public static CommandBase testAuto(DriveSubsystem driveSubsystem)
  {
    
    return new RamseteCommand(null, driveSubsystem::getPosition, driveSubsystem.getRamseteController(), driveSubsystem.get, null, null, null, null, null, null)
  }

  private Autos() {
    //the roborio uses the spi protocol to give the "Maximal Performance" port for the ahrs sensor
    AHRSsensor = new AHRS(SPI.Port.kMXP);
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /*public Autos(){
    //the roborio uses the spi protocol to give the "Maximal Performance" port for the ahrs sensor
    AHRSsensor = new AHRS(SPI.Port.kMXP);
  }*/

  public double GetAvrageAcceleration(){
    //i used pethagoras theoroum to determine the final acceleration
    //note: i am not sure how does it work in navex so if it doesn't work chage "getWorldLinearAccelZ" to "getWorldLinearAccelY" and
    //i the function is times 9.80665^2 because it says the the "gravity component is removed" in the documantation and
    //the data is returned in G force and needed to be converted to m/s^2
    double acceleration = Math.pow(AHRSsensor.getWorldLinearAccelX() *9.80665*9.80665, 2) + Math.pow(AHRSsensor.getWorldLinearAccelZ()*9.80665*9.80665, 2);
    return acceleration;
  }

  public double GetAvrageVelocity(){
    double currentTime = Timer.getFPGATimestamp();
    double deltaTime = currentTime - previousTime;
    previousTime = currentTime;
    //need to be completed
    return 0;
  }


}
