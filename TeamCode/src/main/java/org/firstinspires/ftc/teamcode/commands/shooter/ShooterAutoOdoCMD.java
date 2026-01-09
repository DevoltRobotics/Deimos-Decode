package org.firstinspires.ftc.teamcode.commands.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ShooterAutoOdoCMD extends CommandBase {

    private static InterpLUT VelocityLUT = new InterpLUT();

    static {
        VelocityLUT.add(0, 0);

        VelocityLUT.add(39.5, 1175);
        VelocityLUT.add(70, 1200);
        VelocityLUT.add(99.5, 1280);
        VelocityLUT.add(149.1, 1490); // new
        VelocityLUT.add(157, 1560);

        VelocityLUT.add(1000000000, 10000);


        VelocityLUT.createLUT();
    }

    double distance;
    private final ShooterSubsystem subsystem;

    private final TurretSubsystem turretSubsystem;

    public ShooterAutoOdoCMD(ShooterSubsystem subsystem, TurretSubsystem turretSubsystem) {
        this.subsystem = subsystem;
        this.turretSubsystem = turretSubsystem;
        addRequirements(subsystem);

    }

    @Override
    public void execute(){

        distance = turretSubsystem.getDistanceToGoal();

        double velocity = VelocityLUT.get(distance);

        subsystem.setTargetVelocity(Range.clip(velocity, 1175,1540));


    }
}
