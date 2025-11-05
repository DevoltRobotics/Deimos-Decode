package org.firstinspires.ftc.teamcode.commands.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterAutoLLCMD extends CommandBase {

    private static InterpLUT VelocityLUT = new InterpLUT();

    static {
        VelocityLUT.add(0.12, 1900);
        VelocityLUT.add(0.2, 1670);
        VelocityLUT.add(0.25, 1600);
        VelocityLUT.add(0.39, 1430);
        VelocityLUT.add(0.8, 1250);
        VelocityLUT.add(2.21, 1050);

        VelocityLUT.createLUT();
    }

    private final ShooterSubsystem subsystem;

    private final LLSubsystem LL;

    public ShooterAutoLLCMD(ShooterSubsystem subsystem, LLSubsystem LL) {
        this.subsystem = subsystem;
        this.LL = LL;  
        addRequirements(subsystem);
    }

    @Override
    public void execute(){

        if (LL.result != null && LL.result.isValid()) {
            Double Area = LL.getAllianceTA();
            if(Area != null){
                double velocity = VelocityLUT.get(Area);

                subsystem.setTargetVelocity(Range.clip(velocity, 0,1620));
            }
        }
    }
}
