package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class HookSubsystem extends SubsystemBase {

    public Servo hook;

    public HookSubsystem(HardwareMap hMap){
        hook = hMap.get(Servo.class,"gancho");
        hook.setDirection(Servo.Direction.REVERSE);

    }

    public void hookUp(){
        hook.setPosition(0.2);
    }

    public void hookDown(){
        hook.setPosition(0.1);
    }
}
