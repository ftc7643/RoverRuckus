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

@TeleOp(name = "RRDriveFinal3", group = "Competition")

public class RRDriveFinal3 extends OpMode {

// SET DESCRIPTION FOR EACH MOTOR ON DRIVE TRAIN

    private DcMotor LeftDrive = null;
    private DcMotor RightDrive = null;

    private DcMotor BrushMotor1 = null;
    private DcMotor ArmMotor = null;
//    private DcMotor BrushMotor2 = null; // removed 2nd brush motor from robot

// SET DESCRIPTION FOR SERVO

    private Servo Servo1 = null;

// SET DESCRIPTION FOR DISTANCE SENSOR

    private DistanceSensor SensorRange;

// SET DESCRIPTION FOR COLOR SENSOR

    private ColorSensor colorSensor;

// PROGRAM START POINT

    @Override
    public void init () {

// HARDWARE MAPPING IS USED TO MAP LABELS TO EACH MOTOR AND SERVO

        LeftDrive = hardwareMap.get(DcMotor.class,"LeftMotorDrive");
        RightDrive = hardwareMap.get(DcMotor.class,"RightMotorDrive");
        BrushMotor1 = hardwareMap.get(DcMotor.class, "BrushMotor1");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
//        BrushMotor2 = hardwareMap.get(DcMotor.class, "BrushMotor2"); // removed 2nd brush motor from robot

// SERVO

        Servo1 = hardwareMap.servo.get("Servo1");

// HARDWARE MAP FOR DISTANCE SENSOR

        SensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

// HARDWARE MAP FOR COLOR SENSOR

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

// SET DIRECTION OF MOTOR DRIVE / SERVO

        LeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RightDrive.setDirection(DcMotor.Direction.REVERSE);

        BrushMotor1.setDirection(DcMotor.Direction.FORWARD);
//        BrushMotor2.setDirection(DcMotor.Direction.FORWARD); // removed 2nd brush motor from
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);

        Servo1.setDirection(Servo.Direction.FORWARD);

// SETUP TELEMETRY - SENDING DATA TO DRIVER PHONE

        telemetry.addData("Status", "Initialized");

// RESET SERVO TO 0.0

        Servo1.setPosition(0.0);

// TURN ON LED LIGHT ON COLOR SENSOR

        colorSensor.enableLed(true);

// START CONTROL LOOP

    }

    @Override
    public void loop () {

        double LeftPower = 0;
        double RightPower = 0;
        int BrushPower; // Added by Mark
        double ArmPower = 0;


// ON GAME PAD 1 - LEFT STICK CONTROLS FORWARD AND BACK MOTION
// ON GAME PAD 1 - RIGHT STICK CONTROLS TURNS TO THE LEFT OR RIGHT


        double drive = gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x;
        double damping = 1 - 0.8 * turn;
        double lift = gamepad2.left_stick_y;

        BrushPower = gamepad2.right_bumper?1:gamepad2.left_bumper?-1:0;

        BrushMotor1.setPower(BrushPower); // Added by Mark
        // BrushMotor2.setPower(BrushPower); // removed 2nd brush motor from robot

        // BrushMotor1 = gamepad2.left_stick_y;
        // BrushMotor2 = gamepad2.right_stick_y;

        // BrushMotor1 = Range.clip(-1.0, 1.0);
        // BrushMotor2 = Range.clip(-1.0, 1.0);

        LeftPower = Range.clip(drive - turn, -1.0, 1.0) * damping;
        RightPower = Range.clip(drive + turn, -1.0, 1.0) * damping;
        ArmPower = Range.clip(lift, -1.0, 1.0);

        LeftDrive.setPower(LeftPower);
        RightDrive.setPower(RightPower);

        ArmMotor.setPower(ArmPower);


// SEND VALUES TO SERVOS

        if (gamepad2.left_bumper) Servo1.setPosition(0.5);
        if (gamepad2.right_bumper) Servo1.setPosition(1.0);

// SETUP FOR COLOR SENSOR - HSV = HUE SATURATION VALUE MORE ACCURATE THAN USING RGB.

        float hsvValues[] = {0F,0F,0F};
        // final float values[] = hsvValues;

// CONVERT RGB VALUES TO HSV VALUES.

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

// TELEMETRY FOR DEBUGGING - WILL SHOW UP ON DRIVERS PHONE

        telemetry.addData("Text", "***Robot Data***");
        telemetry.addData("left pwr", "left pwr: " + String.format(Locale.US, "%.2f", LeftPower));
        telemetry.addData("right pwr", "right pwr: " + String.format(Locale.US,"%.2f", RightPower));

        telemetry.addData("deviceName",SensorRange.getDeviceName() );
        telemetry.addData("range", String.format(Locale.US, "%.01f cm", SensorRange.getDistance(DistanceUnit.CM)));

        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();

    }

    @Override
    public void stop () {
    }

}


