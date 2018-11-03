package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;

// SELECT TELEOP / AUTONOMOUS - FOR NAME ON DRIVER PHONE SELECTION

@TeleOp(name = "RRDriveFinal5", group = "Competition")

public class RRDrive extends OpMode {

// SET DESCRIPTION FOR EACH MOTOR ON DRIVE TRAIN

    private DcMotor LeftDrive = null;
    private DcMotor RightDrive = null;

    private DcMotor BrushMotor = null;
    private DcMotor ArmMotor = null;

// SET DESCRIPTION FOR SERVO

    private Servo IntakeServo = null;

// SET DESCRIPTION FOR DISTANCE SENSOR

    private DistanceSensor SensorRange;

// SET DESCRIPTION FOR COLOR SENSOR

    private ColorSensor ColorSensor;

// PROGRAM START POINT

    @Override
    public void init () {

// HARDWARE MAPPING IS USED TO MAP LABELS TO EACH MOTOR AND SERVO

        LeftDrive = hardwareMap.get(DcMotor.class,"LeftMotorDrive");
        RightDrive = hardwareMap.get(DcMotor.class,"RightMotorDrive");
        BrushMotor = hardwareMap.get(DcMotor.class, "BrushMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");


// SERVO

        IntakeServo = hardwareMap.servo.get("IntakeServo");

// HARDWARE MAP FOR DISTANCE SENSOR

        SensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

// HARDWARE MAP FOR COLOR SENSOR

        ColorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

// SET DIRECTION OF MOTOR DRIVE / SERVO

        LeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RightDrive.setDirection(DcMotor.Direction.REVERSE);

        BrushMotor.setDirection(DcMotor.Direction.FORWARD);

        ArmMotor.setDirection(DcMotor.Direction.REVERSE);

        IntakeServo.setDirection(Servo.Direction.FORWARD);

// SETUP TELEMETRY - SENDING DATA TO DRIVER PHONE

        telemetry.addData("Status", "Initialized");

// RESET SERVO TO 0.0

        IntakeServo.setPosition(0.0);

// TURN ON LED LIGHT ON COLOR SENSOR

        ColorSensor.enableLed(true);

// START CONTROL LOOP

    }

    @Override
    public void loop () {

        double LeftPower;
        double RightPower;
        double BrushPower;
        double ArmPower;
        
// ON GAME PAD 1 - LEFT STICK CONTROLS FORWARD AND BACK MOTION
// ON GAME PAD 1 - RIGHT STICK CONTROLS TURNS TO THE LEFT OR RIGHT
        
        double drive = gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x;
        double damping = 1 - 0.7 * Math.abs(turn);
        double lift = gamepad2.left_stick_y;

        BrushPower = gamepad2.right_bumper?1:gamepad2.left_bumper?-0.5:0;
        BrushMotor.setPower(BrushPower);
        
        LeftPower = Range.clip(drive - turn, -1.0, 1.0) * damping;
        RightPower = Range.clip(drive + turn, -1.0, 1.0) * damping;
        
        ArmPower = Range.clip(lift, -1.0, 1.0);

        LeftDrive.setPower(LeftPower);
        RightDrive.setPower(RightPower);

        ArmMotor.setPower(ArmPower /2);
        
// SEND VALUES TO SERVOS


        if (gamepad2.x) IntakeServo.setPosition(0.0);
        if (gamepad2.y) IntakeServo.setPosition(1.0);

// SETUP FOR COLOR SENSOR - HSV = HUE SATURATION VALUE MORE ACCURATE THAN USING RGB.

        float hsvValues[] = {0F,0F,0F};
        // final float values[] = hsvValues;

// CONVERT RGB VALUES TO HSV VALUES.

        Color.RGBToHSV(ColorSensor.red() * 8, ColorSensor.green() * 8, ColorSensor.blue() * 8, hsvValues);

// TELEMETRY FOR DEBUGGING - WILL SHOW UP ON DRIVERS PHONE

        telemetry.addData("Text", "***Robot Data***");
        telemetry.addData("left pwr", "left pwr: " + String.format(Locale.US, "%.2f", LeftPower));
        telemetry.addData("right pwr", "right pwr: " + String.format(Locale.US,"%.2f", RightPower));

        telemetry.addData("deviceName",SensorRange.getDeviceName() );
        telemetry.addData("range", String.format(Locale.US, "%.01f cm", SensorRange.getDistance(DistanceUnit.CM)));

        telemetry.addData("Clear", ColorSensor.alpha());
        telemetry.addData("Red  ", ColorSensor.red());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();

    }

    @Override
    public void stop () {
    }

}


