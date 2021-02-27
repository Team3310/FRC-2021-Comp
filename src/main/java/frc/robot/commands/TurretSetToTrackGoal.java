/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretSetToTrackGoal extends CommandBase
{
    private final Turret turret;
    private double turrAngleFromLocation;

    public TurretSetToTrackGoal(Turret subsystem, double turrAngleFromLocation)
    {
        this.turret = subsystem;
        this.turrAngleFromLocation = turrAngleFromLocation;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        turret.setLimelightTurretTrackMode(turrAngleFromLocation);
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
