/*
 * Copyright (c) 2020.  [Bradley Abelman, Matthew Kaboolian, David Stekol]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Name: Team6438TeleOp
 * Purpose: This class contains maps for all hardware on the robot
 * To reference it you need to create an instance of the Team6438HardwareMap class
 * Author: Matthew Batkiewicz
 * Contributors: Bradley Abelman, David Stekol
 * Creation: 8/29/19
 * Last Edit: 8/29/19
 * This is for our test robot - not to be used for real robot.
 */

package org.firstinspires.ftc.teamcode.prototypes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotMovements;
@Disabled
@Deprecated
@TeleOp(name = "!Team 6438 Test Chassis TeleOp", group = "Team 6438 Driver Controlled")
public class Team6438TeleOpCurrent extends RobotMovements {
    //Declare the robot

    @Override
    public void runOpMode() throws InterruptedException {
        //init the hardware
        initRobot(hardwareMap);
        //Telemetry
        telemetry.addData("Hardware Status:", "Mapped");

/*
        //Set zero power behavior
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            //Gamepad 1 variables
            double LSY1 = gamepad1.left_stick_y;
            double RSY1 = gamepad1.right_stick_y;

            boolean A1 = gamepad1.a;
            boolean B1 = gamepad1.b;
            boolean X1 = gamepad1.x;
            boolean Y1 = gamepad1.y;

            //Gamepad 2 variables
            double LSY2 = gamepad2.left_stick_y;
            double RSY2 = gamepad2.right_stick_y;

            boolean A2 = gamepad2.a;
            boolean B2 = gamepad2.b;
            boolean X2 = gamepad2.x;
            boolean Y2 = gamepad2.y;

            boolean LB2 = gamepad2.left_bumper;
            boolean RB2 = gamepad2.right_bumper;

            double LT2 = gamepad2.left_trigger;
            double RT2 = gamepad2.right_trigger;

            boolean DPD2 = gamepad2.dpad_down;
            boolean DPU2 = gamepad2.dpad_up;
            boolean DPR2 = gamepad2.dpad_right;
            boolean DPL2 = gamepad2.dpad_left;


            motorSetPower(- LSY1 / 2, robot.FL);
            motorSetPower(- RSY1 / 2, robot.FR);

            //Plate mover logic
            servoMovement(A1, B1, .4, .93, robot.leftFoundation, robot.rightFoundation);

            //intake motor button logic
            hControlIntakeMove(X1, Y1);


            //gripper logic
            servoMovement(A2, B2, .45, .77, robot.clamp);

            //Move gripper slightly with D Pad
            servoMovement(DPR2, DPL2, robot.clamp.getPosition() + 0.01, robot.clamp.getPosition() - 0.01, robot.clamp);


            //elbow logic
            if (LT2 > 0.01) {
                motorMovementEncoder((int) (LT2 * 100), 1, robot.elbowMotor);

            } else if (RT2 > 0.01) {
                motorMovementEncoder((int) (RT2 * - 100), 1, robot.elbowMotor);
            }

            //wrist logic
            servoMovement(LB2, RB2, 0, 1, robot.wrist);

            //move wrist slightly with D Pad
            servoMovement(DPU2, DPD2, robot.wrist.getPosition() + 0.01, robot.wrist.getPosition() - 0.01, robot.wrist);

            //drive logic for the intake could need to be reversed
            motorSetPower(- LSY2, robot.intakeLeft);
            motorSetPower(- RSY2, robot.intakeRight);

            //Telemetry
            telemetry.addData("Left Foundation", robot.leftFoundation.getPosition());
            telemetry.addData("Right Foundation", robot.rightFoundation.getPosition());
            telemetry.addData("Left Power", robot.FL.getPower());
            telemetry.addData("Right Power", robot.FR.getPower());
            telemetry.addData("Elbow Encoder", robot.elbowMotor.getCurrentPosition());
            telemetry.addData("Wrist Servo Position", robot.wrist.getPosition());
            telemetry.addData("Gripper Servo Position", robot.clamp.getPosition());

        }

 */
    }
}
