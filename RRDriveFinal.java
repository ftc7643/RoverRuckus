package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;

// SELECT TELEOP / AUTONOMOUS - FOR NAME ON DRIVER PHONE SELECTION

@TeleOp(name = "RRDriveFinal", group = "Competition")

public class RRDriveFinal extends OpMode {

// SET DESCRIPTION FOR EACH MOTOR ON DRIVE TRAIN

    private DcMotor LeftDrive;
    private DcMotor RightDrive;

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

        LeftDrive = hardwareMap.dcMotor.get("LeftMotorDrive");
        RightDrive = hardwareMap.dcMotor.get("RightMotorDrive");

// SERVO

        Servo1 = hardwareMap.servo.get("Servo1");

// HARDWARE MAP FOR DISTANCE SENSOR

        SensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

// HARDWARE MAP FOR COLOR SENSOR

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

// SET DIRECTION OF MOTOR DRIVE / SERVO

        LeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RightDrive.setDirection(DcMotor.Direction.REVERSE);
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

// ON GAME PAD 1 - LEFT STICK CONTROLS LEFT MOTOR
// ON GAME PAD 1 - RIGHT STICK CONTROLS RIGHT MOTOR

        float Left = gamepad1.left_stick_y;
        float Right = gamepad1.right_stick_y;

        float LeftMotorDrive = gamepad1.left_stick_y;
        float RightMotorDrive = gamepad1.right_stick_y;

// CLIP THE MOTOR VALUES SO THE VALUES NEVER EXCEED +/- 1

        LeftMotorDrive = Range.clip(LeftMotorDrive, -1, 1);
        RightMotorDrive = Range.clip(RightMotorDrive, -1, 1);

// SEND SPEED VALUES TO MOTORS

        LeftDrive.setPower(LeftMotorDrive / 2);
        RightDrive.setPower(RightMotorDrive / 2);

// SEND DIRECTION TO MOTORS

        LeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RightDrive.setDirection(DcMotor.Direction.FORWARD);

// SEND VALUES TO SERVOS

        if (gamepad2.left_bumper) Servo1.setPosition(0.5);
        if (gamepad2.right_bumper) Servo1.setPosition(1.0);

// SETUP FOR COLOR SENSOR - HSV = HUE SATURATION VALUE MORE ACCURATE THAN USING RGB.

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;

// CONVERT RGB VALUES TO HSV VALUES.

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

// TELEMETRY FOR DEBUGGING - WILL SHOW UP ON DRIVERS PHONE

        telemetry.addData("Text", "***Robot Data***");
        telemetry.addData("left pwr", "left pwr: " + String.format(Locale.US, "%.2f", Left));
        telemetry.addData("right pwr", "right pwr: " + String.format(Locale.US,"%.2f", Right));

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


