/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Turret;

public class TurretSetToGoalAngle extends ExtraTimeoutCommand
{
    private final Turret turret;
    private final Drive drive;

    public TurretSetToGoalAngle(Turret subsystem, Drive drive)
    {
        this.turret = subsystem;
        this.drive = drive;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        turret.setPositionToGoalAngle(drive);
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && turret.hasFinishedTrajectory()) {
            return true;
        }
        return false;
    }
}
