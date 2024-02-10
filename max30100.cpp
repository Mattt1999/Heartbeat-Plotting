/*
 * max30100.cpp
 *
 *  Created on: Nov 7, 2023
 *      Author: matei
 */

/*
 MIT License

 Copyright (c) 2017 Raivis Strogonovs (https://morf.lv)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.


 */

#include "max30100.h"

MAX30100::MAX30100(Mode mode, SamplingRate samplingRate,
		LEDPulseWidth pulseWidth, LEDCurrent IrLedCurrent, bool highResMode,
		bool debug, int i2c_file_descriptor) {
	this->debug = debug;
	this->i2c_fd = i2c_file_descriptor;

	currentPulseDetectorState = PULSE_IDLE;

	setMode(mode);

	//Check table 8 in datasheet on page 19. You can't just throw in sample rate and pulse width randomly. 100hz + 1600us is max for that resolution
	setSamplingRate(samplingRate);
	setLEDPulseWidth(pulseWidth);

	redLEDCurrent = (uint8_t) STARTING_RED_LED_CURRENT;
	lastREDLedCurrentCheck = 0;

	this->IrLedCurrent = IrLedCurrent;
	setLEDCurrents(redLEDCurrent, IrLedCurrent);
	setHighresModeEnabled(highResMode);

	dcFilterIR.w = 0;
	dcFilterIR.result = 0;

	dcFilterRed.w = 0;
	dcFilterRed.result = 0;

	lpbFilterIR.v[0] = 0;
	lpbFilterIR.v[1] = 0;
	lpbFilterIR.result = 0;

	meanDiffIR.index = 0;
	meanDiffIR.sum = 0;
	meanDiffIR.count = 0;

	valuesBPM[0] = 0;
	valuesBPMSum = 0;
	valuesBPMCount = 0;
	bpmIndex = 0;

	irACValueSqSum = 0;
	redACValueSqSum = 0;
	samplesRecorded = 0;
	pulsesDetected = 0;
	currentSaO2Value = 0;

	lastBeatThreshold = 0;

}

pulseoxymeter_t MAX30100::update() {
	pulseoxymeter_t result = {
	/*bool pulseDetected*/false,
	/*float heartBPM*/0.0,
	/*float irCardiogram*/0.0,
	/*float irDcValue*/0.0,
	/*float redDcValue*/0.0,
	/*float SaO2*/currentSaO2Value,
	/*uint32_t lastBeatThreshold*/0,
	/*float dcFilteredIR*/0.0,
	/*float dcFilteredRed*/0.0 };

	fifo_t rawData = readFIFO();

	dcFilterIR = dcRemoval((float) rawData.rawIR, dcFilterIR.w, ALPHA);
	dcFilterRed = dcRemoval((float) rawData.rawRed, dcFilterRed.w, ALPHA);

	float meanDiffResIR = meanDiff(dcFilterIR.result, &meanDiffIR);
	lowPassButterworthFilter(meanDiffResIR/*-dcFilterIR.result*/, &lpbFilterIR);

	irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
	redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
	samplesRecorded++;

	if (detectPulse(lpbFilterIR.result) && samplesRecorded > 0) {
		result.pulseDetected = true;
		pulsesDetected++;

		float ratioRMS = log(sqrt(redACValueSqSum / samplesRecorded))
				/ log(sqrt(irACValueSqSum / samplesRecorded));

		//This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration
		currentSaO2Value = 110.0 - 18.0 * ratioRMS;
		result.SaO2 = currentSaO2Value;

		if (pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0) {
			irACValueSqSum = 0;
			redACValueSqSum = 0;
			samplesRecorded = 0;
		}
	}

	balanceIntesities(dcFilterRed.w, dcFilterIR.w);

	result.heartBPM = currentBPM;
	result.irCardiogram = lpbFilterIR.result;
	result.irDcValue = dcFilterIR.w;
	result.redDcValue = dcFilterRed.w;
	result.lastBeatThreshold = lastBeatThreshold;
	result.dcFilteredIR = dcFilterIR.result;
	result.dcFilteredRed = dcFilterRed.result;

	return result;
}


////modify this to get the timestamp
uint64_t MAX30100::millis() {
	struct timeval tv1;
	gettimeofday(&tv1, NULL);

	return tv1.tv_usec % 1000;
}


bool MAX30100::detectPulse(float sensor_value) {
	static float prev_sensor_value = 0;
	static uint8_t values_went_down = 0;
	static long int currentBeat = 0;
	static long int lastBeat = 0;
	//checks if the rawBeat value is a good one. I.E. currentBeat > lastBeat
	//static bool vector_filled = false;

	struct timespec tim;

	if (abs(sensor_value) > PULSE_MAX_THRESHOLD) {
		currentPulseDetectorState = PULSE_IDLE;
		prev_sensor_value = 0;
		lastBeat = 0;
		currentBeat = 0;
		values_went_down = 0;
		lastBeatThreshold = 0;
		return false;
	}

	switch (currentPulseDetectorState) {
	case PULSE_IDLE:
		if (sensor_value >= PULSE_MIN_THRESHOLD) {
			currentPulseDetectorState = PULSE_TRACE_UP;
			values_went_down = 0;
		}
		break;

	case PULSE_TRACE_UP:
		if (sensor_value > prev_sensor_value) {
			clock_gettime(CLOCK_MONOTONIC, &tim);
			currentBeat = tim.tv_nsec;
			lastBeatThreshold = sensor_value;
		} else {

			if (debug == true) {
				std::cout << "Peak reached: " << sensor_value << " "
						<< prev_sensor_value << std::endl;
			}

			long int beatDuration = currentBeat - lastBeat;
			lastBeat = currentBeat;

			float rawBPM = 0;
			if (beatDuration > 0) //recompute the stuff
			{
				rawBPM = 60000.0 / (float) beatDuration;
				//beatDuration is measured in nano seconds -> according to the formula above, we need it in ms
				rawBPM *= 1000000;
				rawBPM /= 4;

//				if(rawBPM > 70 && rawBPM < 180)
//				{
					//Idea: divide the rawBPM by 10 - get nicer values;
					//push first 10 elems - calibration
					if (bpmIndex < 10) {
						valuesBPM[bpmIndex] = rawBPM;
						valuesBPMSum += valuesBPM[bpmIndex];
						bpmIndex++;
						cout << "rawBPM: " << rawBPM << endl;
					}
					if(bpmIndex == 10)
					{
						vector_filled = true;
						cout << "Sum is "<<valuesBPMSum << endl;
					}
//				}
			}
			if (debug == true && beatDuration > 0) {
				;
			} else {
				cout << "last beat bigger than current " << endl;
			}

			//This method sometimes glitches, it's better to go through whole moving average everytime
			//IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
			//valuesBPMSum -= valuesBPM[bpmIndex];
			//valuesBPM[bpmIndex] = rawBPM;
			//valuesBPMSum += valuesBPM[bpmIndex];

			if(vector_filled == true && beatDuration > 0)
			{
				cout << "rawBPM=" << rawBPM << endl;
				cout << "vector filled" << endl;

				if(rawBPM > 70 && rawBPM < 200)
				{
					valuesBPMSum -= valuesBPM[0];

					for(int i = 0; i < 10; i++)
					{
						valuesBPM[i] = valuesBPM[i+1];
					}
					valuesBPM[9] = rawBPM;
					valuesBPMSum = valuesBPMSum + valuesBPM[9];

					for(int i = 0; i < 10; i++)
					{
						cout << "valuesBPM["<<i<<"]=" << valuesBPM[i] << endl;
					}

					cout <<"Sum=" <<valuesBPMSum<<endl;
					currentBPM = valuesBPMSum / 10;
				}
			}

			if (debug == true) {
				//          Serial.print("AVg. BPM: ");
				//          Serial.println(currentBPM);
				//cout << "currentBPM " << currentBPM << endl;
			}

			currentPulseDetectorState = PULSE_TRACE_DOWN;

			return true;
		}
		break;

	case PULSE_TRACE_DOWN:
		if (sensor_value < prev_sensor_value) {
			values_went_down++;
		}

		if (sensor_value < PULSE_MIN_THRESHOLD) {
			currentPulseDetectorState = PULSE_IDLE;
		}
		break;
	}

	prev_sensor_value = sensor_value;
	return false;
}

void MAX30100::balanceIntesities(float redLedDC, float IRLedDC) {

	if (millis() - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS) {
		//Serial.println( redLedDC - IRLedDC );
		if (IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF
				&& redLEDCurrent < MAX30100_LED_CURRENT_50MA) {
			redLEDCurrent++;
			setLEDCurrents(redLEDCurrent, IrLedCurrent);
			if (debug == true)
				//Serial.println("RED LED Current +");
				cout << "RED LED Current +";
		} else if (redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF
				&& redLEDCurrent > 0) {
			redLEDCurrent--;
			setLEDCurrents(redLEDCurrent, IrLedCurrent);
			if (debug == true)
				//Serial.println("RED LED Current -");
				cout << "RED LED Current +";
		}

		lastREDLedCurrentCheck = millis();
	}
}

// Writes val to address register on device
void MAX30100::writeRegister(byte address, byte val) {
//  Wire.beginTransmission(MAX30100_DEVICE); // start transmission to device
//  Wire.write(address); // send register address
//  Wire.write(val); // send value to write
//  Wire.endTransmission(); // end transmission
	i2c_smbus_write_byte_data(i2c_fd, address, val);
}

uint8_t MAX30100::readRegister(uint8_t address) {
//  Wire.beginTransmission(MAX30100_DEVICE);
//  Wire.write(address);
//  Wire.endTransmission(false);
//  Wire.requestFrom(MAX30100_DEVICE, 1);
//
//  return Wire.read();

	//look into the register's dimention = 8 bits
	__s32 res;
	res = i2c_smbus_read_byte_data(i2c_fd, address);
	return res;
}

// Reads num bytes starting from address register on device in to _buff array
void MAX30100::readFrom(byte address, int num, byte _buff[]) {
	__s32 res;

	int i = 0;
	for (i = 0; i < num; i++) {
		res = i2c_smbus_read_byte_data(i2c_fd, address);
		_buff[++i] = res;
	}

}

void MAX30100::setMode(Mode mode) {
	byte currentModeReg = readRegister( MAX30100_MODE_CONF);
	writeRegister( MAX30100_MODE_CONF, (currentModeReg & 0xF8) | mode);
}

void MAX30100::setHighresModeEnabled(bool enabled) {
	uint8_t previous = readRegister(MAX30100_SPO2_CONF);
	if (enabled) {
		writeRegister(MAX30100_SPO2_CONF, previous | MAX30100_SPO2_HI_RES_EN);
	} else {
		writeRegister(MAX30100_SPO2_CONF, previous & ~MAX30100_SPO2_HI_RES_EN);
	}
}

void MAX30100::setSamplingRate(SamplingRate rate) {
	byte currentSpO2Reg = readRegister( MAX30100_SPO2_CONF);
	writeRegister( MAX30100_SPO2_CONF, (currentSpO2Reg & 0xE3) | (rate << 2));
}

void MAX30100::setLEDPulseWidth(LEDPulseWidth pw) {
	byte currentSpO2Reg = readRegister( MAX30100_SPO2_CONF);
	writeRegister( MAX30100_SPO2_CONF, (currentSpO2Reg & 0xFC) | pw);
}

void MAX30100::setLEDCurrents(byte redLedCurrent, byte IRLedCurrent) {
	writeRegister( MAX30100_LED_CONF, (redLedCurrent << 4) | IRLedCurrent);
}

float MAX30100::readTemperature() {
	byte currentModeReg = readRegister( MAX30100_MODE_CONF);
	writeRegister( MAX30100_MODE_CONF, currentModeReg | MAX30100_MODE_TEMP_EN);

	//delay(100); //This can be changed to a while loop, there is an interrupt flag for when temperature has been read.

	int8_t temp = (int8_t) readRegister( MAX30100_TEMP_INT);
	float tempFraction = (float) readRegister( MAX30100_TEMP_FRACTION) * 0.0625;

	return (float) temp + tempFraction;
}

fifo_t MAX30100::readFIFO() {
	fifo_t result;

	byte buffer[4];
	readFrom( MAX30100_FIFO_DATA, 4, buffer);
	result.rawIR = (buffer[0] << 8) | buffer[1];
	result.rawRed = (buffer[2] << 8) | buffer[3];

	return result;
}

dcFilter_t MAX30100::dcRemoval(float x, float prev_w, float alpha) {
	dcFilter_t filtered;
	filtered.w = x + alpha * prev_w;
	filtered.result = filtered.w - prev_w;

	return filtered;
}

void MAX30100::lowPassButterworthFilter(float x,
		butterworthFilter_t *filterResult) {
	filterResult->v[0] = filterResult->v[1];

	//Fs = 100Hz and Fc = 10Hz
	filterResult->v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult->v[0]);

	//Fs = 100Hz and Fc = 4Hz
	//filterResult->v[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * filterResult->v[0]); //Very precise butterworth filter

	//Fs = 10000Hz and Fc=100Hz
	//filterResult->v[1] = (3.046874709125380054e-2 * x) + (0.93906250581749239892 * filterResult->v[0]);

	filterResult->result = filterResult->v[0] + filterResult->v[1];
}

float MAX30100::meanDiff(float M, meanDiffFilter_t *filterValues) {
	float avg = 0;

	filterValues->sum -= filterValues->values[filterValues->index];
	filterValues->values[filterValues->index] = M;
	filterValues->sum += filterValues->values[filterValues->index];

	filterValues->index++;
	filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

	if (filterValues->count < MEAN_FILTER_SIZE)
		filterValues->count++;

	avg = filterValues->sum / filterValues->count;
	return avg - M;
}

void MAX30100::printRegisters() {
//  Serial.println(readRegister(MAX30100_INT_STATUS),  HEX);
//  Serial.println(readRegister(MAX30100_INT_ENABLE),  HEX);
//  Serial.println(readRegister(MAX30100_FIFO_WRITE), HEX);
//  Serial.println(readRegister(MAX30100_FIFO_OVERFLOW_COUNTER), HEX);
//  Serial.println(readRegister(MAX30100_FIFO_READ), HEX);
//  Serial.println(readRegister(MAX30100_FIFO_DATA),   HEX);
//  Serial.println(readRegister(MAX30100_MODE_CONF), HEX);
//  Serial.println(readRegister(MAX30100_SPO2_CONF), HEX);
//  Serial.println(readRegister(MAX30100_LED_CONF),  HEX);
//  Serial.println(readRegister(MAX30100_TEMP_INT),   HEX);
//  Serial.println(readRegister(MAX30100_TEMP_FRACTION),   HEX);
//  Serial.println(readRegister(MAX30100_REV_ID),      HEX);
//  Serial.println(readRegister(MAX30100_PART_ID),     HEX);
}

