package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
public class SpindexSubsystem extends SubsystemBase {

    public static double IntakePos = 0.02;
    public static double ShootPos = 0.51;

    public static double IntakePos2 = 0.26;

    public static double ShootPos2 = 0.74;

    public static double IntakePos3 = 0.49;

    public static double ShootPos3 = 0.98;


    public static double SpindexDelayFactorSeconds = 0.6;

    public double AngleError;
    DigitalChannel laser;

    Servo spindex;

    public boolean BPresence = false;

    public NormalizedColorSensor colorSensor;
    AnalogInput spindexEncoder;
    public double SpindexPos;


    public static PIDFCoefficients SPcoeffs = new PIDFCoefficients(
            0.5,   // P
            0.00,   // I
            0.015,  // D
            0.00    // F
    );



    public static double TRIGGER_COOLDOWN_MS = 250;             // anti rebote tiempo
    public static double MIN_ADVANCE_RAD = Math.toRadians(110); // avance mínimo para indexar

    private double targetPos = IntakePos;

    private boolean Shootmode = false;     // TRUE = modo manual/tiro; FALSE = modo indexar auto
    public static int nBalls = 0; // CONTAODR INTERNO
    private double deltaT;

    public enum DetectedColor {PURPLE, GREEN, UNKNOWN}

    double NextPosRad = Math.toRadians(120);

    public boolean FirstInitIn = true;

    public boolean FirstInitSho = true;

    //funciones para CMDS

    public void setnBalls(int nBalls){
        SpindexSubsystem.nBalls = nBalls;
    }

    public boolean getFirstInitIn() {
        return FirstInitIn;
    }

    public void setFirstInitIn(boolean FirstInitIn) {
        this.FirstInitIn = FirstInitIn;
    }

    public boolean getFirstInitSho() {
        return FirstInitSho;
    }

    public void setFirstInitsho(boolean FirstInitSho) {
        this.FirstInitSho = FirstInitSho;
    }

    public void setShootmode(boolean Shootmode) {
        this.Shootmode = Shootmode;
    }

    public void holdShootmode() {
        Shootmode = true;
    }

    public boolean getShootmode() {
        return Shootmode;
    }

    public double advanceOneIndex() {
        double lastTargetPos = targetPos;

        if (!Shootmode) {
            if (targetPos == IntakePos) {
                targetPos = IntakePos2;
            } else if (targetPos == IntakePos2) {
                targetPos = IntakePos3;
            } else if (targetPos == IntakePos3) {
                targetPos = IntakePos;
            }
        } else {
            if (targetPos == ShootPos) {
                targetPos = ShootPos2;
            } else if (targetPos == ShootPos2) {
                targetPos = ShootPos3;
            } else if (targetPos == ShootPos3) {
                targetPos = ShootPos;
            }

        }

        return targetPos - lastTargetPos;
    }

    public void returnOneIndex() {
        if (!Shootmode) {
            if (targetPos == IntakePos) {
                targetPos = IntakePos3;
            } else if (targetPos == IntakePos2) {
                targetPos = IntakePos;
            } else if (targetPos == IntakePos3) {
                targetPos = IntakePos2;
            }
        } else {
            if (targetPos == ShootPos) {
                targetPos = ShootPos3;
            } else if (targetPos == ShootPos2) {
                targetPos = ShootPos;
            } else if (targetPos == ShootPos3) {
                targetPos = ShootPos2;
            }

        }
    }

    public int getnBalls() {
        return nBalls;
    }

    public void addnBalls() {
        nBalls++;
    }

    public void lessBalls() {
        nBalls--;
    }

    public void resetnBalls() {
        nBalls = 0;
    }

    public void setTargetPos(double targetPosRad) {
        this.targetPos = targetPosRad;
    }

    public double getTargetPos() {
        return targetPos;
    }

    public boolean getBPresence() {
        return BPresence;
    }

    public double getAngleDiff(double a, double b) {
        return Math.abs(MathFunctions.getSmallestAngleDifference(a, b));
    }

    public SpindexSubsystem(HardwareMap hMap) {
        laser = hMap.get(DigitalChannel.class, "laser");
        laser.setMode(DigitalChannel.Mode.INPUT);

        spindex = hMap.get(Servo.class, "spindex");


        spindexEncoder = hMap.get(AnalogInput.class, "Spencoder");

        colorSensor = hMap.get(NormalizedColorSensor.class, "colorL");
        colorSensor.setGain(8);
    }

    @Override
    public void periodic() {

        if (nBalls == 3) {
            Shootmode = true;
        }

        if (nBalls == 0) {
            Shootmode = false;
        }

        if (nBalls < 0) {
            nBalls = 0;
        } else if (nBalls > 3) {
            nBalls = 3;
        }


        BPresence = laser.getState();

        SpindexPos = (spindexEncoder.getVoltage() / 3.3) * 2 * Math.PI;


        spindex.setPosition(targetPos);


        FtcDashboard.getInstance().getTelemetry().addData("shootMode", Shootmode);
        FtcDashboard.getInstance().getTelemetry().addData("spindex crudo", SpindexPos);
        FtcDashboard.getInstance().getTelemetry().addData("target pos sp", targetPos);
        FtcDashboard.getInstance().getTelemetry().addData("angleerr", AngleError);
        FtcDashboard.getInstance().getTelemetry().addData("presencia", BPresence);
        FtcDashboard.getInstance().getTelemetry().addData("nBalls", nBalls);
    }
}
