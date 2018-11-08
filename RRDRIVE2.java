//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(
        name = "RRDRIVE2",
        group = "Competition"
)
public class RRDRIVE2 extends OpMode {

    DcMotor LeftFront;
    DcMotor RightFront;
    DcMotor LeftRear;
    DcMotor RightRear;
    DcMotor LiftMotor;
    Servo LeftArm = null;
    Servo RightArm = null;

    public RRDRIVE2() {
    }

    public void init() {
        this.LeftFront = (DcMotor)this.hardwareMap.dcMotor.get("LeftFrontDrive");
        this.RightFront = (DcMotor)this.hardwareMap.dcMotor.get("RightFrontDrive");
        this.LeftRear = (DcMotor)this.hardwareMap.dcMotor.get("LeftRearDrive");
        this.RightRear = (DcMotor)this.hardwareMap.dcMotor.get("RightRearDrive");
        this.LiftMotor = (DcMotor)this.hardwareMap.dcMotor.get("ArmLiftMotor");
        this.LeftArm = (Servo)this.hardwareMap.servo.get("LeftArm");
        this.RightArm = (Servo)this.hardwareMap.servo.get("RightArm");
        this.LeftFront.setDirection(Direction.FORWARD);
        this.RightFront.setDirection(Direction.FORWARD);
        this.LeftRear.setDirection(Direction.FORWARD);
        this.RightRear.setDirection(Direction.FORWARD);
        this.LiftMotor.setDirection(Direction.REVERSE);
        this.RightArm.setDirection(Servo.Direction.REVERSE);
        this.telemetry.addData("Status", "Initialized");
        ElapsedTime eTime = new ElapsedTime();
        this.LeftArm.setPosition(0.5D);
        eTime.reset();

        while(eTime.time() < 2.0D) {
            ;
        }

        this.RightArm.setPosition(0.5D);
    }

    public void loop() {
        float gamepad1LeftY = -this.gamepad1.left_stick_x;
        float gamepad1LeftX = this.gamepad1.left_stick_y;
        float gamepad1RightX = this.gamepad1.right_stick_x;
        float gamepad2LeftX = this.gamepad2.left_stick_y;
        float FrontLeft = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
        float FrontRight = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
        float BackLeft = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
        float BackRight = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
        FrontLeft = Range.clip(FrontLeft, -1.0F, 1.0F);
        FrontRight = Range.clip(FrontRight, -1.0F, 1.0F);
        BackLeft = Range.clip(BackLeft, -1.0F, 1.0F);
        BackRight = Range.clip(BackRight, -1.0F, 1.0F);
        float MotorLift = Range.clip(gamepad2LeftX, -1.0F, 1.0F);
        this.LeftFront.setPower((double)(FrontLeft / 3.0F));
        this.LeftRear.setPower((double)(BackLeft / 3.0F));
        this.RightFront.setPower((double)(FrontRight / 3.0F));
        this.RightRear.setPower((double)(BackRight / 3.0F));
        this.LiftMotor.setPower((double)MotorLift);
        this.telemetry.addData("Text", "*** Robot Data***");
        this.telemetry.addData("Joy XL YL XR", String.format("%.2f", gamepad1LeftX) + " " + String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightX));
        this.telemetry.addData("f left pwr", "front left  pwr: " + String.format("%.2f", FrontLeft));
        this.telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
        this.telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
        this.telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));
        if (this.gamepad2.left_bumper) {
            this.LeftArm.setPosition(0.4D);
        }

        if ((double)this.gamepad2.left_trigger > 0.5D) {
            this.LeftArm.setPosition(0.6D);
        }

        if (this.gamepad2.right_bumper) {
            this.RightArm.setPosition(0.4D);
        }

        if ((double)this.gamepad2.right_trigger > 0.5D) {
            this.RightArm.setPosition(0.6D);
        }

        if (this.gamepad2.a) {
            this.LeftArm.setPosition(0.4D);
            this.RightArm.setPosition(0.4D);
        }

        if (this.gamepad2.b) {
            this.LeftArm.setPosition(0.6D);
            this.RightArm.setPosition(0.6D);
        }

    }

    public void stop() {
    }

    double scaleInput(double dVal) {
        double[] scaleArray = new double[]{0.0D, 0.05D, 0.09D, 0.1D, 0.12D, 0.15D, 0.18D, 0.24D, 0.3D, 0.36D, 0.43D, 0.5D};
        int index = (int)(dVal * 16.0D);
        if (index < 0) {
            index = -index;
        }

        if (index > 16) {
            index = 16;
        }

        double dScale = 0.0D;
        if (dVal < 0.0D) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}

