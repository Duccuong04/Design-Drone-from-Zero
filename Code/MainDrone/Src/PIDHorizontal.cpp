#include "PIDHorizontal.h"

PID_Horizontal::PID_Horizontal() {
	roll = {0};
	pitch = {0};
	yaw = {0};
	throttleSetpoint = 0;
}

PID_Horizontal::~PID_Horizontal() {
}

void PID_Horizontal::setKgainConstantRoll(float thePgainRoll, float theIgainRoll, float theDgainRoll, float theMaxOutputValue) {
	roll.Pgain = thePgainRoll;
	roll.Igain = theIgainRoll;
	roll.Dgain = theDgainRoll;
	roll.maxOutputValue = theMaxOutputValue;
}

void PID_Horizontal::setKgainConstantPitch(float thePgainPitch, float theIgainPitch, float theDgainPitch, float theMaxOutputValue) {
	pitch.Pgain = thePgainPitch;
	pitch.Igain = theIgainPitch;
	pitch.Dgain = theDgainPitch;
	pitch.maxOutputValue = theMaxOutputValue;
}

void PID_Horizontal::setKgainConstantYaw(float thePgainYaw, float theIgainYaw, float theDgainYaw, float theMaxOutputValue) {
	yaw.Pgain = thePgainYaw;
	yaw.Igain = theIgainYaw;
	yaw.Dgain = theDgainYaw;
	yaw.maxOutputValue = theMaxOutputValue;
}

void PID_Horizontal::setLevelAdjust(float theLevelAdjustRoll, float theLevelAdjustPitch) {
	// Không còn dùng levelAdjust trong vòng trong,
	// nhưng giữ hàm này để tránh lỗi biên dịch nếu main.cpp có gọi.
	roll.levelAdjust = theLevelAdjustRoll;
	pitch.levelAdjust = theLevelAdjustPitch;
}

// Replace calculateOuterLoop and calculatePID with this improved version

// Outer loop: angle → rate setpoint (unchanged logic but kept here for completeness)
// Outer loop: angle → rate setpoint
void PID_Horizontal::calculateOuterLoop(float measuredRollAngle, float measuredPitchAngle, float dt) {
    const float maxAngle = 30.0f; // ° nghiêng tối đa
    const int rollCenter  = 1484; // giá trị trung tâm thực đo
    const int pitchCenter = 1489;

    // Map joystick PWM → góc mong muốn
    float desiredRollAngle  = ((roll.setPointBase  - rollCenter)  / 500.0f) * maxAngle;
    float desiredPitchAngle = ((pitch.setPointBase - pitchCenter) / 500.0f) * maxAngle;

    const float anglePgain = 6.0f; // giống như bạn đang nhân *6.0f trước đó

    // Sai số góc
    float rollError  = desiredRollAngle  - measuredRollAngle;
    float pitchError = desiredPitchAngle - measuredPitchAngle;

    // Angle error → rate setpoint (deg/s)
    roll.setPoint  = rollError  * anglePgain;
    pitch.setPoint = pitchError * anglePgain;

    // Giới hạn
    const float maxRate = 200.0f;
    if (roll.setPoint  >  maxRate) roll.setPoint  =  maxRate;
    if (roll.setPoint  < -maxRate) roll.setPoint  = -maxRate;
    if (pitch.setPoint >  maxRate) pitch.setPoint =  maxRate;
    if (pitch.setPoint < -maxRate) pitch.setPoint = -maxRate;
}


//// Inner loop: rate PID
//void PID_Horizontal::calculatePID() {
//
//	/*
//		 * Tính setPoint từ tay cầm (RC input)
//		 * RC input (µs) → setPoint (deg/s).
//		 * Tính setpoint (điểm đặt) cho từng trục
//		 * Khởi tạo điểm đặt roll = 0 trước, rồi mới quyết định lệch bao nhiêu tùy theo cần điều khiển.
//	     * “Deadband” 16 µs quanh trung tâm:
//	     * Vùng 1510–1526 coi như “vào số 0”, để loại rung/nhiễu tay cầm. Nếu lệch về phải (lớn hơn tâm), lấy phần vượt quá 1526; nếu lệch về trái (nhỏ hơn tâm), lấy phần thiếu so với 1510.
//		 */
//		// Chuyển đổi setPointRoll sang đơn vị dps để cùng kiểu dữ liệu với inputRoll
//		// Chuyển đổi đơn vị bằng cách chia giá trị cho 3 ta được giá trị lớn nhất của setPointRoll ( (500-8)/3 = 164d/s ).
//		roll.setPoint = 0;
//		// Thiết lập khoảng deadBand là 16us để đạt được độ ổn định
//		if (roll.setPointBase > 1526)
//		{
//			roll.setPoint = roll.setPointBase - 1526;
//		}
//		else if (roll.setPointBase < 1480)
//		{
//			roll.setPoint  =  roll.setPointBase - 1480;
//		}
//
//		roll.setPoint -= roll.levelAdjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
//		roll.setPoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.
//
//
//		//The PID set point in degrees per second is determined by the pitch receiver input.
//		//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
//		pitch.setPoint = 0;
//		//We need a little dead band of 16us for better results.
//		if (pitch.setPointBase > 1511)
//		{
//			pitch.setPoint = pitch.setPointBase - 1511;
//		}
//		else if (pitch.setPointBase < 1480)
//		{
//			pitch.setPoint = pitch.setPointBase - 1480;
//		}
//
//		pitch.setPoint -= pitch.levelAdjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
//		pitch.setPoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.
//
//		//The PID set point in degrees per second is determined by the yaw receiver input.
//		//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
//		yaw.setPoint = 0;
//		//We need a little dead band of 16us for better results.
//		if (throttleSetpoint > 1200) { //Do not yaw when turning off the motors.
//			if (yaw.setPointBase > 1514)
//			{
//				yaw.setPoint = (yaw.setPointBase - 1514) / 3.0;
//			}
//			else if (yaw.setPointBase < 1480)
//			{
//				yaw.setPoint = (yaw.setPointBase - 1480) / 3.0;
//			}
//		}
//	// =======================
//	// ROLL PID
//	// =======================
//	roll.momentError = roll.setPoint - roll.inputValue;
//
//	// P
//	float rollPterm = roll.Pgain * roll.momentError;
//
//	// I (anti-windup)
//	bool atPidLimit = fabsf(roll.outputValue) >= (roll.maxOutputValue - 1.0f);
//	bool sameDir    = (roll.momentError * roll.outputValue) > 0;
//	if (!atPidLimit || !sameDir) {
//		roll.ImemValue += roll.Igain * roll.momentError;
//	} else {
//		roll.ImemValue *= 0.99f;
//	}
//
//	// D (derivative on measurement + LPF)
//	static float prevInputRoll = 0.0f;
//	static float rollDfiltered = 0.0f;
//	float dInputRoll = roll.inputValue - prevInputRoll;
//	prevInputRoll = roll.inputValue;
//	rollDfiltered = 0.9f * rollDfiltered + 0.1f * dInputRoll;
//	float rollDterm = -roll.Dgain * rollDfiltered;
//
//	roll.outputValue = rollPterm + roll.ImemValue + rollDterm;
//
//	if (roll.outputValue >  roll.maxOutputValue) roll.outputValue =  roll.maxOutputValue;
//	if (roll.outputValue < -roll.maxOutputValue) roll.outputValue = -roll.maxOutputValue;
//
//	// =======================
//	// PITCH PID
//	// =======================
//	pitch.momentError = pitch.setPoint - pitch.inputValue;
//
//	// P
//	float pitchPterm = pitch.Pgain * pitch.momentError;
//
//	// I (anti-windup)
//	atPidLimit = fabsf(pitch.outputValue) >= (pitch.maxOutputValue - 1.0f);
//	sameDir    = (pitch.momentError * pitch.outputValue) > 0;
//	if (!atPidLimit || !sameDir) {
//		pitch.ImemValue += pitch.Igain * pitch.momentError;
//	} else {
//		pitch.ImemValue *= 0.99f;
//	}
//
//	// D (derivative on measurement + LPF)
//	static float prevInputPitch = 0.0f;
//	static float pitchDfiltered = 0.0f;
//	float dInputPitch = pitch.inputValue - prevInputPitch;
//	prevInputPitch = pitch.inputValue;
//	pitchDfiltered = 0.9f * pitchDfiltered + 0.1f * dInputPitch;
//	float pitchDterm = -pitch.Dgain * pitchDfiltered;
//
//	pitch.outputValue = pitchPterm + pitch.ImemValue + pitchDterm;
//
//	if (pitch.outputValue >  pitch.maxOutputValue) pitch.outputValue =  pitch.maxOutputValue;
//	if (pitch.outputValue < -pitch.maxOutputValue) pitch.outputValue = -pitch.maxOutputValue;
//
//	// =======================
//	// YAW PID
//	// =======================
//	yaw.momentError = yaw.setPoint - yaw.inputValue;
//
//	float yawPterm = yaw.Pgain * yaw.momentError;
//
//	atPidLimit = fabsf(yaw.outputValue) >= (yaw.maxOutputValue - 1.0f);
//	sameDir    = (yaw.momentError * yaw.outputValue) > 0;
//	if (!atPidLimit || !sameDir) {
//		yaw.ImemValue += yaw.Igain * yaw.momentError;
//	} else {
//		yaw.ImemValue *= 0.99f;
//	}
//
//	// Giữ nguyên D-term theo error cho yaw
//	float yawDterm = yaw.Dgain * (yaw.momentError - yaw.previousError);
//	yaw.previousError = yaw.momentError;
//
//	yaw.outputValue = yawPterm + yaw.ImemValue + yawDterm;
//
//	if (yaw.outputValue >  yaw.maxOutputValue) yaw.outputValue =  yaw.maxOutputValue;
//	if (yaw.outputValue < -yaw.maxOutputValue) yaw.outputValue = -yaw.maxOutputValue;
//}

void PID_Horizontal::calculatePID(float dt) {
    if (dt <= 0.0f || dt > 0.05f) dt = 0.004f; // bảo vệ: giả sử 250 Hz

    // ====== Low-pass filter config cho D-term ======
    const float tau = 0.08f;  // ~1/(2π*tau) ≈ 2.6Hz corner, tune sau
    const float alpha = tau / (tau + dt);

    /*
    		 * Tính setPoint từ tay cầm (RC input)
    		 * RC input (µs) → setPoint (deg/s).
    		 * Tính setpoint (điểm đặt) cho từng trục
    		 * Khởi tạo điểm đặt roll = 0 trước, rồi mới quyết định lệch bao nhiêu tùy theo cần điều khiển.
    	     * “Deadband” 16 µs quanh trung tâm:
    	     * Vùng 1510–1526 coi như “vào số 0”, để loại rung/nhiễu tay cầm. Nếu lệch về phải (lớn hơn tâm), lấy phần vượt quá 1526; nếu lệch về trái (nhỏ hơn tâm), lấy phần thiếu so với 1510.
    		 */
    		// Chuyển đổi setPointRoll sang đơn vị dps để cùng kiểu dữ liệu với inputRoll
    		// Chuyển đổi đơn vị bằng cách chia giá trị cho 3 ta được giá trị lớn nhất của setPointRoll ( (500-8)/3 = 164d/s ).
    		roll.setPoint = 0;
    		// Thiết lập khoảng deadBand là 16us để đạt được độ ổn định
    		if (roll.setPointBase > 1526)
    		{
    			roll.setPoint = roll.setPointBase - 1526;
    		}
    		else if (roll.setPointBase < 1488)
    		{
    			roll.setPoint  =  roll.setPointBase - 1488;
    		}

    		roll.setPoint -= roll.levelAdjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
    		roll.setPoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


    		//The PID set point in degrees per second is determined by the pitch receiver input.
    		//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    		pitch.setPoint = 0;
    		//We need a little dead band of 16us for better results.
    		if (pitch.setPointBase > 1511)
    		{
    			pitch.setPoint = pitch.setPointBase - 1511;
    		}
    		else if (pitch.setPointBase < 1483)
    		{
    			pitch.setPoint = pitch.setPointBase - 1483;
    		}

    		pitch.setPoint -= pitch.levelAdjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
    		pitch.setPoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

    		//The PID set point in degrees per second is determined by the yaw receiver input.
    		//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    		yaw.setPoint = 0;
    		//We need a little dead band of 16us for better results.
    		if (throttleSetpoint > 1200) { //Do not yaw when turning off the motors.
    			if (yaw.setPointBase > 1514)
    			{
    				yaw.setPoint = (yaw.setPointBase - 1514) / 3.0;
    			}
    			else if (yaw.setPointBase < 1480)
    			{
    				yaw.setPoint = (yaw.setPointBase - 1480) / 3.0;
    			}
    		}

    // ======================= ROLL =======================
       roll.momentError = roll.setPoint - roll.inputValue;

       // --- P ---
       float rollPterm = roll.Pgain * roll.momentError;

       // --- I (anti-windup + clamp) ---
       roll.ImemValue += roll.Igain * roll.momentError * dt;
       float Icap_roll = roll.maxOutputValue * 0.5f;
       if (roll.ImemValue > Icap_roll) roll.ImemValue = Icap_roll;
       if (roll.ImemValue < -Icap_roll) roll.ImemValue = -Icap_roll;
       if (fabsf(roll.momentError) < 0.3f) roll.ImemValue *= 0.995f; // decay khi error nhỏ

       // --- D (derivative on measurement, filtered) ---
       static float prevInputRoll = 0.0f;
       static float rollDfiltered = 0.0f;
       float dInputRoll = (roll.inputValue - prevInputRoll) / dt;
       prevInputRoll = roll.inputValue;
       rollDfiltered = alpha * rollDfiltered + (1.0f - alpha) * dInputRoll;
       float rollDterm = -roll.Dgain * rollDfiltered;

       roll.outputValue = rollPterm + roll.ImemValue + rollDterm;
       if (roll.outputValue >  roll.maxOutputValue) roll.outputValue =  roll.maxOutputValue;
       if (roll.outputValue < -roll.maxOutputValue) roll.outputValue = -roll.maxOutputValue;

       // ======================= PITCH =======================
       pitch.momentError = pitch.setPoint - pitch.inputValue;

       float pitchPterm = pitch.Pgain * pitch.momentError;

       pitch.ImemValue += pitch.Igain * pitch.momentError * dt;
       float Icap_pitch = pitch.maxOutputValue * 0.5f;
       if (pitch.ImemValue > Icap_pitch) pitch.ImemValue = Icap_pitch;
       if (pitch.ImemValue < -Icap_pitch) pitch.ImemValue = -Icap_pitch;
       if (fabsf(pitch.momentError) < 0.3f) pitch.ImemValue *= 0.995f;

       static float prevInputPitch = 0.0f;
       static float pitchDfiltered = 0.0f;
       float dInputPitch = (pitch.inputValue - prevInputPitch) / dt;
       prevInputPitch = pitch.inputValue;
       pitchDfiltered = alpha * pitchDfiltered + (1.0f - alpha) * dInputPitch;
       float pitchDterm = -pitch.Dgain * pitchDfiltered;

       pitch.outputValue = pitchPterm + pitch.ImemValue + pitchDterm;
       if (pitch.outputValue >  pitch.maxOutputValue) pitch.outputValue =  pitch.maxOutputValue;
       if (pitch.outputValue < -pitch.maxOutputValue) pitch.outputValue = -pitch.maxOutputValue;

       // ======================= YAW =======================
       yaw.momentError = yaw.setPoint - yaw.inputValue;

       float yawPterm = yaw.Pgain * yaw.momentError;

       yaw.ImemValue += yaw.Igain * yaw.momentError * dt;
       float Icap_yaw = yaw.maxOutputValue * 0.5f;
       if (yaw.ImemValue > Icap_yaw) yaw.ImemValue = Icap_yaw;
       if (yaw.ImemValue < -Icap_yaw) yaw.ImemValue = -Icap_yaw;
       if (fabsf(yaw.momentError) < 0.5f) yaw.ImemValue *= 0.995f;

       // D-term yaw (Betaflight thường bỏ qua hoặc rất nhỏ)
       float yawDterm = yaw.Dgain * (yaw.momentError - yaw.previousError) / dt;
       yaw.previousError = yaw.momentError;

       yaw.outputValue = yawPterm + yaw.ImemValue + yawDterm;
       if (yaw.outputValue >  yaw.maxOutputValue) yaw.outputValue =  yaw.maxOutputValue;
       if (yaw.outputValue < -yaw.maxOutputValue) yaw.outputValue = -yaw.maxOutputValue;
}
