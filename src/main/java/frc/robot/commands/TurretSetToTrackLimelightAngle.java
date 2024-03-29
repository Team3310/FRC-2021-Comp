/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretSetToTrackLimelightAngle extends CommandBase
{
    private final Turret turret;
    private double limelightOffsetAngleDeg;
    private boolean isActive;

    public TurretSetToTrackLimelightAngle(Turret subsystem, double limelightOffsetAngleDeg, boolean isActive)
    {
        this.turret = subsystem;
        this.limelightOffsetAngleDeg = limelightOffsetAngleDeg;
        this.isActive = isActive;

 //       addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if(isActive){
            if (limelightOffsetAngleDeg < -100.0) {
                limelightOffsetAngleDeg = turret.getTurretToGoalOffsetAngle();
            }
            turret.setLimelightTrackMode(limelightOffsetAngleDeg);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Turret interrupted = " + interrupted);
    }
}
