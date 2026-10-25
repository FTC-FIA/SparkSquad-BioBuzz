package org.firstinspires.ftc.teamcode.base;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Drive;

public abstract class OrangeBotBaseOpMode extends LinearOpMode {

    protected DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    protected GoBildaPinpointDriver pinpoint;
    protected Limelight3A limelight;
    protected Drive drive;

    /**
     * Configures the hardware that all OpModes using this robot share.
     * Call this first thing on startup.
     */
    public void config() {
        // Instantiate hardware
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");
        intake = hardwareMap.dcMotor.get("intake_motor");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure hardware
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        pinpoint.setOffsets(-7.375, -5.875, DistanceUnit.INCH);

        drive = new Drive(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                pinpoint,
                telemetry
        );
    }

}
