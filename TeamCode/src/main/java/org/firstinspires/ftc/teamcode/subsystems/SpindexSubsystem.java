package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pattern;

@Config
public class SpindexSubsystem extends SubsystemBase {

    public static Pattern obeliskPattern = Pattern.PGP;

    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;

    float normRed, normBlue, normGreen;

    public enum DetectedColor {
        Purple,
        Green,
        Unknown
    }

    private ElapsedTime loopTime = new ElapsedTime();

    public DetectedColor detectedColor = DetectedColor.Unknown;
    public double ticks_per_rev = 8192;
    public boolean isShooting = false;
    public static double IntakePos = 60;

    public static double ShootPos = 0;

    public double GrenBallPos = 0;


    public static double ShootPos2 = 120;


    public static double ShootPos3 = 240;

    public static double tolerance = 2.5;


    DigitalChannel laser;
    DigitalChannel home;

    DcMotor Spindex;

    public PIDFController SpinPID;
    public static double kS = 0;

    public static PIDFCoefficients Spincoeffs = new PIDFCoefficients(0.012, 0, 0.0006, 0);

    public static double TRIGGER_COOLDOWN_MS = 100;             // anti rebote tiempo
    private double targetPos;
    private boolean Shootmode = false;     // TRUE = modo manual/tiro; FALSE = modo indexar auto
    public static int nBalls = 0; // CONTAODR INTERNO

    public boolean FirstInitIn = true;

    public boolean FirstInitSho = true;

    double power = 0;

    double distCm = 10;

    static double spindexEncoderOffset = 0;
    double spindexPosDeg = 0;

    public static double minO = 0.087;

    double ff;


    private double lastError;
    public boolean shouldPulse = true;
    private ElapsedTime stuckDetectionTimer = new ElapsedTime();

    public SpindexSubsystem(HardwareMap hMap) {
        laser = hMap.get(DigitalChannel.class, "laser");
        laser.setMode(DigitalChannel.Mode.INPUT);

        home = hMap.get(DigitalChannel.class, "home");
        home.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hMap.get(NormalizedColorSensor.class, "color");
        colorSensor.setGain(3);

        Spindex = hMap.get(DcMotor.class, "spindex");
        Spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spindex.setDirection(DcMotorSimple.Direction.REVERSE);


        setTargetPos(IntakePos);
        SpinPID = new PIDFController(Spincoeffs);
        SpinPID.setTolerance(2);
        SpinPID.setMinimumOutput(minO);
    }

    @Override
    public void periodic() {
        ff = Math.signum(SpinPID.getPositionError()) * kS;

        distCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        SpinPID.setMinimumOutput(minO);
        SpinPID.setCoefficients(Spincoeffs);

        if (nBalls == 3 && !isShooting) {
            Shootmode = true;
        } else if (nBalls == 0 && !isShooting) {
            Shootmode = false;
        }

        nBalls = Range.clip(nBalls, 0, 3);

        SpinPID.setSetPoint(targetPos);

        spindexPosDeg = ((Spindex.getCurrentPosition() - spindexEncoderOffset) / ticks_per_rev) * 360;

        power = Math.min(SpinPID.calculate(spindexPosDeg), 0.55);

        boolean shouldBeMoving = Spindex.getPower() >= minO;

        if (shouldBeMoving && Math.abs(SpinPID.getPositionError() - lastError) >= tolerance) {
            stuckDetectionTimer.reset();
        }

        if (shouldBeMoving && stuckDetectionTimer.seconds() >= 0.8 && shouldPulse) {
            Spindex.setPower(-1);
        } else if (Math.abs(SpinPID.getPositionError()) < tolerance) {
            Spindex.setPower(0);
        } else {
            Spindex.setPower(power);
        }

        lastError = SpinPID.getPositionError();

        /*colors = colorSensor.getNormalizedColors();

        normRed = colors.red/colors.alpha;
        normBlue = colors.blue/colors.alpha;
        normGreen = colors.green/colors.alpha;

        if (normRed > 0.016 && normBlue > 0.03 && normGreen< 0.05){
            detectedColor = DetectedColor.Purple;
        } else if (normRed < 0.03 && normBlue < 0.09 && normGreen > 0.035) {
            detectedColor = DetectedColor.Green;
        }else {
            detectedColor = DetectedColor.Unknown;
        }*/


        FtcDashboard.getInstance().getTelemetry().addData("shootMode", Shootmode);
        FtcDashboard.getInstance().getTelemetry().addData("spindex target pos", targetPos);
        FtcDashboard.getInstance().getTelemetry().addData("spindexposDeg", spindexPosDeg);
        FtcDashboard.getInstance().getTelemetry().addData("distanciaB", distCm);
        FtcDashboard.getInstance().getTelemetry().addData("nBalls", nBalls);
        FtcDashboard.getInstance().getTelemetry().addData("spinError", SpinPID.getPositionError());
        FtcDashboard.getInstance().getTelemetry().addData("spindex loop time", loopTime.milliseconds());
        FtcDashboard.getInstance().getTelemetry().addData("spindex should be moving", shouldBeMoving);
        FtcDashboard.getInstance().getTelemetry().addData("spindex stuck time", stuckDetectionTimer.seconds());
        FtcDashboard.getInstance().getTelemetry().addData("spindexPower", Spindex.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("blue", normBlue);
        FtcDashboard.getInstance().getTelemetry().addData("green", normGreen);
        FtcDashboard.getInstance().getTelemetry().addData("detected color", detectedColor);
        FtcDashboard.getInstance().getTelemetry().addData("Patron", obeliskPattern);

        loopTime.reset();
    }

    public DetectedColor getDetectedColor() {
        colors = colorSensor.getNormalizedColors();

        normRed = colors.red / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        normGreen = colors.green / colors.alpha;


        if (normRed > 0.016 && normBlue > 0.03 && normGreen < 0.05) {
            return DetectedColor.Purple;
        } else if (normRed < 0.03 && normBlue < 0.09 && normGreen > 0.035) {
            return DetectedColor.Green;
        } else {
            return DetectedColor.Unknown;
        }

    }

    public double getspindexPos() {
        return Spindex.getCurrentPosition();
    }

    public void setnBalls(int nBalls) {
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


    public boolean getShootmode() {
        return Shootmode;
    }

    public void advanceOneIndex() {
        targetPos -= 120;
    }

    public void returnOneIndex() {
        targetPos += 120;
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

    public boolean getBPresence() {
        return (distCm < 6);
    }

    public void setTargetPos(double targetPos) {
        this.targetPos = targetPos;
    }

    public double getTargetPos() {
        return targetPos;
    }

    public void setShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }

    public double getPatternOffset() {
        if (obeliskPattern == Pattern.PGP) {
            return GrenBallPos + 120;
        } else if (obeliskPattern == Pattern.PPG) {
            return GrenBallPos - 120;
        } else if (obeliskPattern == Pattern.GPP) {
            return GrenBallPos;
        } else {
            if (nBalls == 3) {
                return targetPos;
            } else {
                return (targetPos + 60);
            }
        }
    }

    public double getPatternOffsetShootcMode() {
        if (obeliskPattern == Pattern.PGP) {
            return GrenBallPos + 120;
        } else if (obeliskPattern == Pattern.PPG) {
            return GrenBallPos - 120;
        } else if (obeliskPattern == Pattern.GPP) {
            return GrenBallPos;
        } else {
            return (targetPos + 60);
        }
    }

    public void SARSP() {
        spindexEncoderOffset = Spindex.getCurrentPosition();
    }
}