package org.firstinspires.ftc.teamcode.commands.turret;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretPowerCMD extends CommandBase {

    TurretSubsystem subsystem;

    double power;

    public TurretPowerCMD(TurretSubsystem subsystem, double power) {
        this.subsystem = subsystem;
        this.power = power;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setTurretPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setTurretPower(0);
    }
}
