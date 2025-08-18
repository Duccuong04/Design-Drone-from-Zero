/*
 * barometer.h
 *
 *  Created on: Aug 17, 2025
 *      Author: Cuong Nguyen
 */

#ifndef INC_BAROMETER_H_
#define INC_BAROMETER_H_

#include "main.h"

/*
 * Enum mô tả trạng thái kết nối của MS5611
 * Ok
 * Error
 * DeviceNotConnected
 * DevideInvalid
 *
 */
typedef enum  {
	BAROMETER_Result_Ok = 0x00,
	BAROMETER_Result_Error,
	BAROMETER_Result_DeviceNotConnected, // Thiết bị không được kết nối đến
	BAROMETER_Result_DeviceInvalid       // Không phải địa chỉ của mpu6050
} BAROMETER_Result;

/*
 * Class Barometer khai báo biến và hàm đọc dữ liệu
 */
class Barometer {
private :

	I2C_HandleTypeDef* hi2c;

	/*
	 * các hằng số hiệu chuẩn (Calibration coefficients) đọc từ PROM của MS5611.
	 */
	uint16_t dataProm[6];

	/*
	 * các giá trị bù (second order compensation) để cải thiện độ chính xác khi nhiệt độ thấp.
	 */
	int64_t OFF2;
	int64_t SENS2;

	/*
	 * Nhóm dữ liệu đo
	 */

	// giá trị nhiệt độ thô chưa bù.
	uint32_t rawTemperature;
	//giá trị áp suất thô chưa bù.
	uint32_t rawPressure;
	// giá trị áp suất đã bù nhiệt và calib.
	int64_t compensatedPressure;

	/*
	 *  Quản lý trạng thái đọc dữ liệu
	 */

	// trạng thái đọc áp suất (chu kỳ đọc).
	uint8_t stageOfBarometer;
	//trạng thái đọc nhiệt độ (chu kỳ đọc).
	uint8_t stageOfTemperature;

	/*
	 * Lấy trung bình nhiệt độ
	 */

	// con trỏ index vào mảng nhiệt độ quay vòng.
	uint8_t indexAverageTemperatureMem;
	// buffer lưu 6 giá trị nhiệt độ thô.
	uint32_t rawTemperatureRotatingMemory[6];
	// tổng cộng dồn để tính trung bình nhiệt độ.
	uint32_t rawAverageTemperatureTotal;

	/*
	 * Lấy trung bình áp suất
	 */
	// buffer lưu 50 mẫu áp suất.
	int32_t pressureRotatingMemory[50];
	// tổng áp suất (dùng tính trung bình).
	int32_t averagePressureTotal;
	// con trỏ index vào buffer.
	uint8_t indexAveragePressureMem;

	/*
	 * Bộ lọc (Complementary filter)
	 */
	// áp suất tức thời sau lọc
	float actualPressure;
	// thành phần lọc chậm (low-pass).
	float actualPressureSlow;
	// thành phần lọc nhanh (high-pass).
	float actualPressureFast;
	// độ chênh lệch áp suất (dùng phát hiện thay đổi nhanh như rơi dù).
	float actualPressureDiff;

	/*
	 * Biến liên quan đến PID điều khiển độ cao
	 */

	// giá trị ga điều khiển bằng tay.
	int16_t manualThrottle;
	// trạng thái thay đổi độ cao thủ công.
	uint8_t manualAltitudeChange;
	// con trỏ buffer chống rơi (parachute detect).
	uint8_t parachuteRotatingMemLocation;
	// buffer lưu dữ liệu để phát hiện rơi tự do.
	int32_t parachuteBuffer[35], parachuteThrottle;
	// áp suất lần trước để so sánh.
	float pressureParachutePrevious;
	// sai số PID (hệ số gain).
	float pidErrorGainAltitude;
	// sai số tạm thời.
	float pidErrorAltitudeTemp;
	// bộ nhớ tích phân PID
	float pid_Imem_Altitude;
	// giá trị độ cao mong muốn (setpoint)
	float pidAltitudeSetpoint;
	// giá trị đầu vào PID (áp suất/độ cao đo).
	float pidAltitudeInput;
	// giá trị đầu ra PID (điều chỉnh throttle).
	float pidOutputAltitude;
	// sai số D lần trước để tính đạo hàm.
	float pidLastAltitude_D_error;

	/*
	 * reset cảm biến (gửi lệnh reset).
	 */
	HAL_StatusTypeDef Reset();

	/*
	 * đọc các hệ số hiệu chuẩn từ PROM.
	 */
	HAL_StatusTypeDef ReadProm();

	/*
	 * đọc kết quả đo từ lần yêu cầu trước
	 */
	uint32_t GetDataFromPreviousRequest();

	/*
	 * trả về nhiệt độ trung bình từ buffer.
	 */
	uint32_t GetAverageTemperature();

	/*
	 * yêu cầu đo nhiệt độ (non-blocking).
	 */
	void RequestGetTemperatureData();

	/*
	 * yêu cầu đo áp suất (non-blocking).
	 */
	void RequestGetPressureData();

	/*
	 * tính áp suất bù nhiệt từ raw + calib.
	 */
	int64_t CompensatePressure();

	/*
	 * trả về áp suất trung bình từ buffer.
	 */
	float GetAveragePressure();

	/*
	 * lọc dữ liệu áp suất (kết hợp nhanh/chậm).
	 */
	float UseComplementaryFilter();

	/*
	 * phát hiện thay đổi dài hạn (trôi độ cao).
	 */
	void CalculateLongtermChange();

	/*
	 * tính PID điều khiển độ cao.
	 */
	void CalculateAltitudePID(uint8_t TheStateThrottle);

	/*
	 * reset lại biến PID khi đổi mode hoặc reset
	 */
	void ResetValuesOfPID();
public:

	/*
	 * Hệ số PID
	 */
	float pid_Pgain_Altitude;
	float pid_Igain_Altitude;
	float pid_Dgain_Altitude;
	int16_t	pidMaxAltitude;

	/*
	 * thiết lập hệ số PID
	 */

	void SetKgainPID(float thePgain,float theIgain, float theDgain, int16_t theMaxPID);

	/*
	 * đọc sensor, lọc dữ liệu, tính PID
	 */
	void ReadAndCalculatePIDBarometer(uint8_t theFlightMode, uint8_t theTakeoffDetected,uint8_t TheStateThrottle);

	/*
	 * kiểm tra kết nối sensor
	 */
	BAROMETER_Result IsReadyToInterface();

	/*
	 * khởi tạo cảm biến (reset, đọc PROM, cấu hình ban đầu).
	 */
	HAL_StatusTypeDef Init();

	/*
	 * hàm dựng (khởi tạo class với I2C).
	 */
	Barometer(I2C_HandleTypeDef * theI2c);

	/*
	 * lưu áp suất mặt đất (chuẩn 0m).
	 */
	void SetGroundPressure();

	/*
	 * trả về giá trị ga thủ công.
	 */
	int16_t GetManualThrottle();

	/*
	 * trả về áp suất hiện tại sau lọc
	 */
	float GetActualPressure();

	/*
	 * đặt độ cao mong muốn.
	 */
	void Set_PID_altitude_setpoint(float thePidAltitudeSetpoint);

	/*
	 * trả về giá trị điều khiển từ PID để đưa vào ESC (motor).
	 */
	float GetPidOutputAltitude();
};

#endif /* INC_BAROMETER_H_ */
