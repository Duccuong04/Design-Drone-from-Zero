/*
 * GPS.h
 *
 *  Created on: Aug 18, 2025
 *      Author: DELL
 */

#ifndef __GPS_H
#define __GPS_H

#include "main.h"
#include "MySerial.h"



class GPS
{
private:

	/*
	 * cho biết vị trí nằm ở Bắc hay Nam, Đông hay Tây.
	 */
	uint8_t latitudeNorth, longtiudeEast ;

	/*
	 * Đếm số ký tự đã nhận trong 1 bản tin NMEA.
	 */
	uint16_t messageCounter;

	/*
	 * Vĩ độ, kinh độ đã xử lý (dùng trong nội suy và tính toán).
	 */
	int32_t latGPS, lonGPS ;

	/*
	 * Đánh dấu khi kết thúc chuỗi NMEA (khi gặp ký tự *).
	 */
	uint8_t newLineFound;

	/*
	 * Giá trị GPS lần trước, dùng để tính vận tốc/gia tăng.
	 */
	int32_t previousLonGPS,previousLatGPS;

	/*
	 * Byte đọc từ UART, buffer lưu chuỗi NMEA.
	 */
	uint8_t readSerialByte, incommingMessage[100];

	/*
	 * Giá trị GPS thực nhận từ bản tin.
	 */
	int32_t actualLatGPS, actualLonGPS;

	/*
	 * Đã lưu waypoint hay chưa (0 = chưa, 1 = có, 2 = reset).
	 */
	uint8_t waypointSet;

	/*
	 * Bộ đếm nội suy giữa các lần đọc GPS (do GPS chỉ 5Hz, nội suy để có dữ liệu “giả lập” ở 50Hz).
	 */
	int16_t gpsAddCounter;

	/*
	 * Vĩ độ, kinh độ waypoint.
	 */
	int32_t l_latWaypoint, l_lonWaypoint;

	/*
	 * Tín hiệu điều chỉnh pitch/roll theo tọa độ Bắc.
	 */
	float gpsPitchAdjustNorth, gpsRollAdjustNorth;

	/*
	 * Giá trị delta (gia tăng) cho nội suy.
	 * Tích lũy cho nội suy
	 */
	float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;

	/*
	 * Đánh dấu có dữ liệu GPS mới; số lần nội suy còn lại.
	 */
	uint8_t new_gpsDataAvailable, new_gps_dataCounter;

	/*
	 * Vị trí trong bộ nhớ vòng, ring buffer
	 */
	uint8_t gps_rotatingMemLocation;

	/*
	 * Tổng sai số vĩ độ/kinh độ để tính D-term.
	 */
	int32_t gps_latTotalAvarage, gps_lonTotalAvarage;

	/*
	 * Bộ nhớ vòng lưu sai số để lọc/nội suy.
	 */
	int32_t gps_latRotatingMem[40], gps_lonRotatingMem[40];

	/*
	 * Sai số hiện tại so với waypoint.
	 */
	int32_t gps_latError, gps_lonError;

	/*
	 * Sai số lần trước.
	 */
	int32_t gps_latErrorPrevious, gps_lonErrorPrevious;

	/*
	 * Bộ watchdog phát hiện mất tín hiệu GPS.
	 */
	uint32_t gps_watchdogTimer;

	GPIO_TypeDef* port;
	uint16_t pin;

public:
	/*
	 * Số vệ tinh đang dùng.
	 */
	uint8_t numberUsedSats;

	/*
	 * Loại fix (0=none, 2=2D, 3=3D).
	 */
	uint8_t fixType;

	/*
	 * Hệ số PID cho GPS.
	 */
	float Pgain, Dgain;

	/*
	 * Giá trị hiệu chỉnh cuối cùng cho drone.
	 */
	float gpsPitchAdjust, gpsRollAdjust;

	/*
	 * Constructor, khởi tạo với LED báo GPS.
	 */
	GPS(GPIO_TypeDef* thePort, uint16_t thePin);

	/*
	 * Hàm chính đọc dữ liệu GPS, phân tích NMEA, cập nhật trạng thái.
	 */
	void ReadGPS(uint8_t theStateMachine, uint8_t* theError, uint8_t* theFlightMode, float theAngleYaw);

	/*
	 * Trả về vĩ độ/kinh độ hiện tại.
	 */
	int GetLat(void);
	int GetLon(void);

	/*
	 * Khởi tạo GPS (gửi UBX command: tắt GPGSV, set 5Hz, set baud 57600).
	 */
	HAL_StatusTypeDef Init(void);

	/*
	 * Gán UART dùng cho GPS
	 */
	void SetHardWare(UART_HandleTypeDef *huart,USART_TypeDef *UARTx);

	/*
	 * Khởi tạo UART ở baud rate mong muốn.
	 */
	void MX_USART_UART_Init(unsigned int baud);

	/*
	 * Set hệ số PID cho Roll.
	 */
	void setKgainConstantRoll(float thePgainRoll, float theDgainRoll);

};


#endif

