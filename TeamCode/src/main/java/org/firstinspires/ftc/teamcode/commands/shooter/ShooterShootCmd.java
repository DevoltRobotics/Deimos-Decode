package org.firstinspires.ftc.teamcode.commands.shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterShootCmd extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    DoubleSupplier targetvelocity;

    public ShooterShootCmd(ShooterSubsystem shooterSubsystem, DoubleSupplier targetvelocity){
        this.shooterSubsystem = shooterSubsystem;
        this.targetvelocity = targetvelocity;
        addRequirements(shooterSubsystem);
    }

    public ShooterShootCmd(ShooterSubsystem shooterSubsystem, double targetvelocity) {
        this(shooterSubsystem, () -> targetvelocity);
    }

    @Override
    public void execute() {
        shooterSubsystem.setTargetVelocity(targetvelocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.setTargetVelocity(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
