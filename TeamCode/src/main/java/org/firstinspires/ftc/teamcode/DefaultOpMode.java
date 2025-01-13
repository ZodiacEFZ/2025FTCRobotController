/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Default OpMode", group = "Iterative OpMode")
public class DefaultOpMode extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        front_left = hardwareMap.get(DcMotor.class, "frontLeft");
        front_right = hardwareMap.get(DcMotor.class, "frontRight");
        rear_left = hardwareMap.get(DcMotor.class, "rearLeft");
        rear_right = hardwareMap.get(DcMotor.class, "rearRight");

        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        rear_left.setDirection(DcMotorSimple.Direction.FORWARD);
        rear_right.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double front_left_power = (y + x + rx) / denominator;
        double rear_left_power = (y - x + rx) / denominator;
        double front_right_power = (y - x - rx) / denominator;
        double rear_right_power = (y + x - rx) / denominator;

        front_left.setPower(front_left_power);
        rear_left.setPower(rear_left_power);
        front_right.setPower(front_right_power);
        rear_right.setPower(rear_right_power);

        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Gamepad1", "Ly:(%.2f) Lx:(%.2f) RX:(%.2f)", y, x, rx);
        telemetry.addData("Motors", "FL:(%.2f) FR:(%.2f) RL:(%.2f) RR:(%.2f)", front_left_power, front_right_power, rear_left_power, rear_right_power);
    }

    @Override
    public void stop() {
    }
}
