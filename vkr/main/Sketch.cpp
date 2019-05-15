#include <Arduino.h>
#include <Wire.h>
#include <avr/eeprom.h>
#include <Adafruit_ADS1015.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>	

/*
	TODO List
	Рассчет токов как основных характеристик сравнения
	Промежуточное сравнение и сравнение итоговыых значений пока просто вывод значений

	Вывод информации 
	экран степеней отклоенения
	экран по фазе - токи расчетные измеренные и напряжения
		
*/

/************************************************************************
	Правило: у всех массивов отвечающие за обмотки индексы привязаны к следующей связке
	0 = AB
	1 = BC
	2 = AC													
************************************************************************/
#define BUTTON_1 A0
#define BUTTON_2 A1
#define BEEPER A2

#define CONNECTION_TYPE_STAR 0
#define CONNECTION_TYPE_TRIANGLE 1

byte EEMEM eeprom_first_start = 0;
byte EEMEM eeprom_connection_type = CONNECTION_TYPE_STAR; 
byte EEMEM eeprom_gain_amperage = 0;
byte EEMEM eeprom_gain_voltage = 0;
float EEMEM eeprom_impedance_ab = 0;
float EEMEM eeprom_impedance_bc = 0;
float EEMEM eeprom_impedance_ac = 0;
float EEMEM eeprom_voltage_mult_ab = 1;
float EEMEM eeprom_voltage_mult_bc = 1;
float EEMEM eeprom_voltage_mult_ac = 1;
float EEMEM eeprom_amperage_mult_ab = 1;
float EEMEM eeprom_amperage_mult_bc = 1;
float EEMEM eeprom_amperage_mult_ac = 1;

// Measurement mode
#define MM_STOP false	// stop measurement
#define MM_WORK true	// do measurement

// Mode work
// Settings setup
#define MW_NEED_SETUP 0										// first start
#define MW_SETUP_CONNECTION_TYPE 1							// setting winding connection type
#define MW_SETUP_GAIN_AMPERAGE 2							// setting winding connection type
#define MW_SETUP_GAIN_VOLTAGE 3								// setting winding connection type
#define MW_SETUP_MULT_VOLTAGE_AB 4							// setting voltage multiplier value for AB winding
#define MW_SETUP_MULT_VOLTAGE_BC 5							// setting voltage multiplier value for BC winding
#define MW_SETUP_MULT_VOLTAGE_AC 6							// setting voltage multiplier value for AC winding
#define MW_SETUP_MULT_AMPERAGE_AB 7							// setting amperage multiplier value for AB winding
#define MW_SETUP_MULT_AMPERAGE_BC 8							// setting amperage multiplier value for BC winding
#define MW_SETUP_MULT_AMPERAGE_AC 9							// setting amperage multiplier value for AC winding
#define MW_SETUP_IMPEDANCE_AB 10							// setting impedance value for AB winding 
#define MW_SETUP_IMPEDANCE_BC 11							// setting impedance value for BC winding 
#define MW_SETUP_IMPEDANCE_AC 12							// setting impedance value for AC winding 

#define MW_SETUP_START MW_SETUP_CONNECTION_TYPE				// first setup mode for swiping
#define MW_SETUP_STOP MW_SETUP_IMPEDANCE_AC					// last setup mode for swiping

// Showing characteristics
#define MW_SHOW_IMPEDANCE_CRITICLE_ERRORS 13				// showing counters of windings critical errors { amperage error counter AB | amperage error counter BC | amperage error counter AC }
#define MW_SHOW_IMPEDANCE_ERRORS 14							// showing all windings amperage errors { amperage error AB | amperage error BC | amperage error AC }
#define MW_SHOW_IMPEDANCE_AMPERAGE_AB 15					// showing AB winding amperage { measured | perfect}
#define MW_SHOW_IMPEDANCE_AMPERAGE_BC 16					// showing BC winding amperage { measured | perfect}
#define MW_SHOW_IMPEDANCE_AMPERAGE_AC 17					// showing AC winding amperage { measured | perfect}
#define MW_SHOW_IMPEDANCE_WINDING_CHARS_AB 18				// showing AB winding characteristics { voltage | amperage error | avg measured amperage }
#define MW_SHOW_IMPEDANCE_WINDING_CHARS_BC 19				// showing BC winding characteristics { voltage | amperage error | avg measured amperage }
#define MW_SHOW_IMPEDANCE_WINDING_CHARS_AC 20				// showing AC winding characteristics { voltage | amperage error | avg measured amperage }

#define MW_SHOWING_START MW_SHOW_IMPEDANCE_CRITICLE_ERRORS	// first showing mode for swiping
#define MW_SHOWING_STOP MW_SHOW_IMPEDANCE_WINDING_CHARS_AC 	// last showing mode for swiping

#define MW_CONTROLL_MEASUREMENT	18

#define IC_ERROR_CRITICAL 85


/*	
	------------------------------------------------------
	|   Mode name	 | Gain | +/- Voltage | mV per 1 bit |		
	------------------------------------------------------
	| GAIN_TWOTHIRDS | x2/3 |    6.144	  | 0.1875		 | 
	| GAIN_ONE       | x1   |    4.096    | 0.125		 |
	| GAIN_TWO       | x2   |    2.048    | 0.0625		 |
	| GAIN_FOUR      | x4   |    1.024    | 0.03125		 |
	| GAIN_EIGHT     | x8   |    0.512    | 0.015625	 |
	| GAIN_SIXTEEN   | x16  |    0.256    | 0.0078125	 |
	------------------------------------------------------
*/


Adafruit_ADS1115 adsVoltage(0x48);
Adafruit_ADS1115 adsAmperage(0x49);

struct Ads1115 {
	byte currentAmperageGain = 0;
	byte currentVoltageGain = 0;
	float gainStep[6] = {0.1875, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125};
	float voltageStep = 0.1875 / 1000.0;
	float amperageStep = 0.1875 / 1000.0;
};

struct AdsChars {
	float voltage[3] = {0, 0, 0};
	float measuredAmperage[3] = {0, 0, 0};
	float perfectAmperage[3] = {0, 0, 0};
		
	float sumVoltage[3] = {0, 0, 0};
	float sumMeasuredAmperage[3] = {0, 0, 0};
	byte measurementsCount = 50;
	byte currentMeasurement = 1;
};	

struct Settings {
	bool isReadyToWork = false;
	bool connectionType = CONNECTION_TYPE_STAR;
	float criticleError = 2.5;
	float impedance[3] = {0, 0, 0};
	// true - settings | false - characteristics
	bool isSetupMode;
	float voltage[3] = {1, 1, 1};
	float amperage[3] = {1, 1, 1};
};

struct Error {
	// Уровень ошибки текущий за определенное количество измерений
	float curLvl[3] = {0, 0, 0};
	// Счетчик превышение нормы. Пока привязан как выше нормы но не на конкретное значение
	unsigned long criticalLvlCount[3] = {0, 0, 0};
	bool hasAsymmetry = false;
	bool hasIC = false;
};

struct SetDigit {
	float value;
	float curMultiplier;
};

struct ModeWork {
	byte current;
	byte prev;
};


/*
Первая кнопка. 
	Короткое нажатие:
		В режиме настроек увеличение числа
		В режиме измерения смена режима показа вперед
	Длинное нажатие:
		В режиме настроек переход между разрядами
		В режиме измерения переход в режим включения/отключения измерения

Вторая кнопка.
	Короткое нажатие:
		В режиме настроек уменьшение числа
		В режиме измерения смена режима показа назад
	Динное нажатие:
		В режиме измерения в меню критических величин сбрасывает счетчик
		Переход в режим настроек/отображения измерений кроме режима критических величин
*/
OneButton button1Callback(BUTTON_1, false);
OneButton button2Callback(BUTTON_2, false);
AdsChars adsChars;
Settings settings;
Ads1115 ads;
Error icError;
SetDigit setDigit;
LiquidCrystal_I2C lcd(0x27, 16, 2);
ModeWork modeWork;

bool lcdUpdateScreen = false;
	
void initAvgVars();
void getAdsParams();
void saveSettings();
void button2Click();
void button1Click();
void setEditValue();
void processAsMode();
void displayAsMode();
void taskMeasurement();
void showStaticAmperage();
void initMultiplierCoef();
void checkIsReadyToWork();
void initAdsVoltageGain();
void displayStaticAsMode();
void initAdsAmperageGain();
void fillComparedAmperage();
void button1LongPressStart();
void button2LongPressStart();
void showStaticWindingChars();
float getICLevel(float, byte);
void calculateRealAdsParams();
bool isErrorsAsymmetric(float*);
void initImpedanceCriticalValue();
void lcdPrintCriticalLvl(byte _num);
void showWindingCharsValues(byte _num);
void lcdPrinRoundedCurErrorLevel(byte _num);
void setAdsGainByIndex(Adafruit_ADS1115*, byte);
void lcdClearCell(byte col, byte row, byte rowLength);


void setup() {
	Serial.begin(9600);
	Serial.println("Initialization...");
	
	adsVoltage.setGain(GAIN_TWOTHIRDS);
	adsAmperage.setGain(GAIN_TWOTHIRDS);
	
	adsVoltage.begin();
	adsAmperage.begin();
	
	lcd.init();
	lcd.backlight();
	lcd.clear();
	
	button1Callback.attachClick(button1Click);
	button1Callback.attachLongPressStart(button1LongPressStart);
	button1Callback.setDebounceTicks(30);
	button1Callback.setClickTicks(200);
	button1Callback.setPressTicks(1000);
	
	button2Callback.attachClick(button2Click);
	button2Callback.attachLongPressStart(button2LongPressStart);
	button2Callback.setDebounceTicks(30);
	button2Callback.setClickTicks(200);
	button2Callback.setPressTicks(1000);
	
	if (eeprom_read_byte(&eeprom_first_start) != 102) {
		eeprom_update_byte(&eeprom_connection_type, CONNECTION_TYPE_STAR);
		eeprom_update_byte(&eeprom_gain_amperage, 0);
		eeprom_update_byte(&eeprom_gain_voltage, 0);
		eeprom_update_float(&eeprom_impedance_ab, 0);
		eeprom_update_float(&eeprom_impedance_bc, 0);
		eeprom_update_float(&eeprom_impedance_ac, 0);
		eeprom_update_float(&eeprom_voltage_mult_ab, 0);
		eeprom_update_float(&eeprom_voltage_mult_bc, 0);
		eeprom_update_float(&eeprom_voltage_mult_ac, 0);
		eeprom_update_float(&eeprom_amperage_mult_ab, 0);
		eeprom_update_float(&eeprom_amperage_mult_bc, 0);
		eeprom_update_float(&eeprom_amperage_mult_ac, 0);
		eeprom_update_byte(&eeprom_first_start, 102);
	}
	
	ads.currentAmperageGain = eeprom_read_byte(&eeprom_gain_amperage);
	ads.currentVoltageGain = eeprom_read_byte(&eeprom_gain_voltage);
	settings.connectionType = eeprom_read_byte(&eeprom_connection_type);
	settings.impedance[0] = eeprom_read_float(&eeprom_impedance_ab);
	settings.impedance[1] = eeprom_read_float(&eeprom_impedance_bc);
	settings.impedance[2] = eeprom_read_float(&eeprom_impedance_ac);
	settings.multiplierVoltage[0] = eeprom_read_float(&eeprom_voltage_mult_ab);
	settings.multiplierVoltage[1] = eeprom_read_float(&eeprom_voltage_mult_bc);
	settings.multiplierVoltage[2] = eeprom_read_float(&eeprom_voltage_mult_ac);
	settings.multiplierAmperage[0] = eeprom_read_float(&eeprom_amperage_mult_ab);
	settings.multiplierAmperage[1] = eeprom_read_float(&eeprom_amperage_mult_bc);
	settings.multiplierAmperage[2] = eeprom_read_float(&eeprom_amperage_mult_ac);
	
	initAdsVoltageGain();
	initAdsAmperageGain();
	
	settings.criticleError = settings.connectionType == CONNECTION_TYPE_STAR? 2.5f: 5.0f;
	initImpedanceCriticalValue();
	
	initMultiplierCoef();
	checkIsReadyToWork();
	
	pinMode(BEEPER, OUTPUT);
	digitalWrite(BEEPER, LOW);
	
	modeWork.prev = modeWork.current;
	initAvgVars();
	Serial.println("Completed!");
	Serial.println("Stand by...");
	displayStaticAsMode();
	displayAsMode();
}
/*
void initImpedanceCriticalValue() {
	for (byte i = 0; i < 3; i++) {
		impedance.criticalValue[i] = settings.impedance[i] / 100.0f * settings.criticleError;
	}
}*/

void checkIsReadyToWork() {
	bool isReadyToWork = true;
	for(byte i = 0; i < 3; i++) {
		if (settings.impedance[i] == 0 || settings.multiplierAmperage[i] == 0 || settings.multiplierVoltage[i] == 0) {
			isReadyToWork = false;
			break;
		}
	}
		
	settings.isSetupMode = false;
	if (isReadyToWork) {
		modeWork.current = MW_CONTROLL_MEASUREMENT;
	} else {
		modeWork.current = MW_NEED_SETUP;
		Serial.println("Need setup params");
	}
}

void loop() {
	button1Callback.tick();
	button2Callback.tick();
	displayAsMode();
	if(settings.isReadyToWork == MM_WORK) {
		getAdsParams();
		if (adsChars.currentMeasurement < adsChars.measurementsCount) {
			adsChars.currentMeasurement++;
		} else {
			lcdUpdateScreen = true;
			for (byte i = 0; i < 3; i++) {
				adsChars.voltage[i] = adsChars.sumVoltage[i] / adsChars.measurementsCount * settings.multiplierVoltage[i];
				adsChars.measuredAmperage[i] = adsChars.sumMeasuredAmperage[i] / adsChars.measurementsCount * settings.multiplierAmperage[i];
			/*	impedance.measured[i] = impedance.sumMeasured[i] / adsChars.measurementsCount;
				icError.curLvl[i] = getICLevel(impedance.measured[i], i);*/
			}
			initAvgVars();
			
			bool isHasIC = false;
			for (byte i = 0; i < 3; i++) {
				if (icError.curLvl[i] > 100 - settings.connectionType) {
				//	icError.hasAsymmetry = isErrorsAsymmetric(impedance.measured);
					isHasIC = true;
					break;
				}
			}
			if (!icError.hasAsymmetry) {
				isHasIC = false;
			} else {
				settings.isReadyToWork = MM_STOP;
				modeWork.current = MW_SHOW_IMPEDANCE_CRITICLE_ERRORS;
				digitalWrite(BEEPER, HIGH);
				icError.hasIC = true;
			}
		}
	} 
}

/**
 * Получение параметров с АЦП
 */
void getAdsParams() {
	float measuredVoltage[3] = {0, 0, 0};
	float measuredAmperage[3] = {0, 0, 0};
	
	for (byte i = 0; i < 3; i++) {
		measuredVoltage[i] = adsVoltage.readADC_SingleEnded(i);
		measuredAmperage[i] = adsAmperage.readADC_SingleEnded(i);
	}
	for (byte i = 0; i < 3; i++) {
		measuredVoltage[i] *= ads.voltageStep;
		measuredAmperage[i] *= ads.amperageStep;
		
		//measuredImpedance[i] = measuredVoltage[i] / measuredAmperage[i] * multiplier.coef[i];
		
		adsChars.sumVoltage[i] += measuredVoltage[i];
		adsChars.sumMeasuredAmperage[i] += measuredAmperage[i];
	//	impedance.sumMeasured[i] += measuredImpedance[i];
	}
	
	for (byte i = 0; i < 3; i++) {
		if (isErrorsAsymmetric(measuredImpedance)) {
			if (getICLevel(measuredImpedance[i], i) >= IC_ERROR_CRITICAL && icError.criticalLvlCount[i] < 1000) {
				icError.criticalLvlCount[i]++;
			}
		}
	}
}

void initAvgVars() {
	for (byte i = 0; i < 3; i++) {
		adsChars.sumVoltage[i] = 0;
		adsChars.sumMeasuredAmperage[i] = 0;
	//	impedance.sumMeasured[i] = 0;
	}
	adsChars.currentMeasurement = 1;
}

float getICLevel(float value, byte _num) {
	if (settings.impedance[_num] >= value) {
		return 0;
	}
	
	return abs(value - settings.impedance[_num]) * 100.0 / impedance.criticalValue[_num];
}


void initMultiplierCoef() {
	for(byte i = 0; i < 3; i++) {
		multiplier.coef[i] = settings.multiplierVoltage[i] == 0? 0: settings.multiplierVoltage[i] / settings.multiplierAmperage[i];
	}
}

bool isErrorsAsymmetric(float* _impedance) {
	float divImpedance[3] = {0, 0, 0};
	float avgImpedance = 0;
	byte sign = 100;
	for(byte i = 0; i < 3; i++) {
		divImpedance[i] = _impedance[i] - settings.impedance[i];
		if (divImpedance[i] > 0) {
			sign++;
		} else {
			sign--;
		}
		avgImpedance += divImpedance[i];
	}
	if (sign == -97) {
		return false;
	}
	avgImpedance /= 3;
	
	// всего допустимая погрешность 5 процентов вверх и вниз
	float lowBound = avgImpedance * 0.05;
	float highBound = avgImpedance + lowBound;
	lowBound = avgImpedance - lowBound;
	for (byte i = 0; i < 3; i++) {
		if (!(divImpedance[i] >= lowBound && divImpedance[i] <= highBound)) {
			return true;
		}
	}
	return false;
}

void showGainInfo() {
	lcdClearCell(0, 1, 16);
	switch((int)setDigit.value) {
		case 0:
			lcd.print(F("6.144 0.1875"));
			break;
		case 1:
			lcd.print(F("4.096 0.125"));
			break;
		case 2:
			lcd.print(F("1.024 0.0625"));
			break;
		case 3:
			lcd.print(F("0.512 0.03125"));
			break;
		case 4:
			lcd.print(F("0.512 0.015625"));
			break;
		case 5:
			lcd.print(F("0.256 0.0078125"));
			break;
	}
}

void displayAsMode() {
	if (modeWork.current != modeWork.prev) {
		modeWork.prev = modeWork.current;
		lcdUpdateScreen = true;
		displayStaticAsMode();
	}
	if (!lcdUpdateScreen) {
		return;
	} 
	lcdUpdateScreen = false;
	
	switch(modeWork.current) {
		case MW_SETUP_CONNECTION_TYPE:
			lcdClearCell(0, 1, 8);
			if (setDigit.value == CONNECTION_TYPE_STAR) {
				lcd.print(F("Star"));
			} else {
				lcd.print(F("Triangle"));
			}
			break;
		case MW_SETUP_GAIN_AMPERAGE:
		case MW_SETUP_GAIN_VOLTAGE:
			showGainInfo();
			break;
		case MW_SETUP_IMPEDANCE_AB:
		case MW_SETUP_IMPEDANCE_BC:
		case MW_SETUP_IMPEDANCE_AC:
		case MW_SETUP_MULT_VOLTAGE_AB:
		case MW_SETUP_MULT_VOLTAGE_BC:
		case MW_SETUP_MULT_VOLTAGE_AC:
		case MW_SETUP_MULT_AMPERAGE_AB:
		case MW_SETUP_MULT_AMPERAGE_BC:
		case MW_SETUP_MULT_AMPERAGE_AC:
			lcdClearCell(7, 0, 8);
			lcd.print(F("x"));
			lcd.print(setDigit.curMultiplier, 3);
			lcdClearCell(0, 1, 16);
			lcd.print(setDigit.value, 3);
			break;
		case MW_SHOW_IMPEDANCE_CRITICLE_ERRORS:
			lcdClearCell(11, 0, 4);
			lcdPrintCriticalLvl(0);
			lcdClearCell(3, 1, 4);
			lcdPrintCriticalLvl(1);
			lcdClearCell(11, 1, 4);
			lcdPrintCriticalLvl(2);
			break;
		case MW_SHOW_IMPEDANCE_ERRORS:
			lcdClearCell(3, 0, 5);
			lcdPrinRoundedCurErrorLevel(0);
			lcdClearCell(11, 0, 5);
			lcdPrinRoundedCurErrorLevel(1);
			lcdClearCell(3, 1, 5);
			lcdPrinRoundedCurErrorLevel(2);
			break;
		case MW_SHOW_IMPEDANCE_AMPERAGE_AB:
			showWindingCharsValues(0);
			break;
		case MW_SHOW_IMPEDANCE_AMPERAGE_BC:
			showWindingCharsValues(1);
			break;
		case MW_SHOW_IMPEDANCE_AMPERAGE_AC:
			showWindingCharsValues(2);
			break;
		case MW_SHOW_IMPEDANCE_WINDING_CHARS_AB:
			showWindingCharsValues(0);
			break;
		case MW_SHOW_IMPEDANCE_WINDING_CHARS_BC:
			showWindingCharsValues(1);
			break;
		case MW_SHOW_IMPEDANCE_WINDING_CHARS_AC:
			showWindingCharsValues(2);
			break;
	}
}

void lcdPrinRoundedCurErrorLevel(byte _num) {
	if (icError.curLvl[_num] < 1000) {
		lcd.print(round(icError.curLvl[_num]));
	} else {
		lcd.print(999);
		lcd.print(F("+"));
	}
}

void lcdPrintCriticalLvl(byte _num) {
	if (icError.criticalLvlCount[_num] < 1000) {
		lcd.print(icError.criticalLvlCount[_num]);
	} else {
		lcd.print(999);
		lcd.print(F("+"));
	}
}

void displayStaticAsMode() {
	lcd.clear();
	switch(modeWork.current) {
		case MW_SETUP_CONNECTION_TYPE:
			lcd.print(F("Connection type"));
		break;
		case MW_SETUP_GAIN_AMPERAGE:
			lcd.print(F("A gain maxV/step"));
			break;
		case MW_SETUP_GAIN_VOLTAGE:
			lcd.print(F("V gain maxV/step"));
			break;
		case MW_SETUP_IMPEDANCE_AB:
			lcd.print(F("R1 AB"));
			break;
		case MW_SETUP_IMPEDANCE_BC:
			lcd.print(F("R2 BC"));
			break;
		case MW_SETUP_IMPEDANCE_AC:
			lcd.print(F("R3 AC"));
			break;
		case MW_SETUP_MULT_VOLTAGE_AB:
			lcd.print(F("Vm AB"));
			break;
		case MW_SETUP_MULT_VOLTAGE_BC:
			lcd.print(F("Vm BC"));
			break;
		case MW_SETUP_MULT_VOLTAGE_AC:
			lcd.print(F("Vm AC"));
			break;
		case MW_SETUP_MULT_AMPERAGE_AB:
			lcd.print(F("Am AB"));
			break;
		case MW_SETUP_MULT_AMPERAGE_BC:
			lcd.print(F("Am BC"));
			break;
		case MW_SETUP_MULT_AMPERAGE_AC:
			lcd.print(F("Am AC"));
			break;
		case MW_SHOW_IMPEDANCE_CRITICLE_ERRORS:
			lcd.print(F("ECrit "));
			lcd.setCursor(8, 0);
			lcd.print(F("AB="));
			lcd.setCursor(0, 1);
			lcd.print(F("BC="));
			lcd.setCursor(8, 1);
			lcd.print(F("AC="));
			break;
		case MW_SHOW_IMPEDANCE_ERRORS:
			lcd.print(F("R1="));
			lcd.setCursor(8, 0);
			lcd.print(F("R2="));
			lcd.setCursor(0, 1);
			lcd.print(F("R3="));
			lcd.setCursor(14, 1);
			lcd.print(F("e%"));
			break;
		case MW_SHOW_IMPEDANCE_AMPERAGE_AB:
			lcd.print(F("AB"));
			showStaticAmperage();
			break;
		case MW_SHOW_IMPEDANCE_AMPERAGE_BC:
			lcd.print(F("BC"));
			showStaticAmperage();
			break;
		case MW_SHOW_IMPEDANCE_AMPERAGE_AC:
			lcd.print(F("AC"));
			showStaticAmperage();
			break;
		case MW_SHOW_IMPEDANCE_WINDING_CHARS_AB:
			lcd.print(F("AB"));
			showStaticWindingChars();
			break;
		case MW_SHOW_IMPEDANCE_WINDING_CHARS_BC:
			lcd.print(F("BC"));
			showStaticWindingChars();
			break;
		case MW_SHOW_IMPEDANCE_WINDING_CHARS_AC:
			lcd.print(F("AC"));
			showStaticWindingChars();
			break;
		case MW_CONTROLL_MEASUREMENT:
			lcd.print(F("Measurement"));
			lcd.setCursor(0, 1);
			lcd.print(F("B1=work; B2=stop"));
			break;
		case MW_NEED_SETUP:
			lcd.print(F("Oooops!"));
			lcd.setCursor(0, 1);
			lcd.print(F("Setup required"));
			break;
	}
}


void showWindingCharsValues(byte _num) {
	lcdClearCell(6, 0, 10);
	byte roundedSign = 2;
	if (adsChars.measuredAmperage[_num] < 100) {
		roundedSign = 8;
	} else if (adsChars.voltage[_num] < 1000) {
		roundedSign = 7;
	} else {
		roundedSign = 4;
	}
	lcd.print(adsChars.voltage[_num], roundedSign);
	
	
	lcdClearCell(0, 1, 8);
	if (adsChars.measuredAmperage[_num] < 100) {
		roundedSign = 2;
	} else if (adsChars.measuredAmperage[_num] < 1000) {
		roundedSign = 1;
	} else {
		roundedSign = 0;
	}
	if (roundedSign > 0) {
		lcd.print(adsChars.measuredAmperage[_num], roundedSign);
	} else {
		lcd.print(999);
		lcd.print(F("+"));
	}
}

void showStaticAmperage() {
	lcd.setCursor(4, 0);
	lcd.print(F("P"));
	lcd.setCursor(0, 1);
	lcd.print(F("M"));
}

// CR measured|perfect
void showStaticWindingChars() {
	lcd.setCursor(4, 0);
	lcd.print(F("I="));
	lcd.setCursor(0, 1);
	lcd.print(F("V="));
	lcd.setCursor(8, 1);
	lcd.print(F("E="));
}

//
// очистка поля значения от старых данных
void lcdClearCell(byte col, byte row, byte rowLength) {                
	lcd.setCursor(col, row);                                        
	for (byte i = 0; i < rowLength; i++) {                             
		lcd.print(F(" "));
	}
	lcd.setCursor(col, row);                                          
}


float getCurrentWindingImpedanceValue() {
	byte _windingIndex = 0;
	float _voltage = 0;
	float _amperage = 0;
	
	switch(modeWork.current){
		case MW_SETUP_IMPEDANCE_AB:
		_windingIndex = 0;
		break;
		case MW_SETUP_IMPEDANCE_BC:
		_windingIndex = 1;
		break;
		case MW_SETUP_IMPEDANCE_AC:
		_windingIndex = 2;
		break;
	}

	_voltage = adsVoltage.readADC_SingleEnded(_windingIndex);
	_amperage = adsAmperage.readADC_SingleEnded(_windingIndex);

	_voltage *= ads.voltageStep;
	_amperage *= ads.amperageStep;
	
	return _amperage == 0? 0: _voltage / _amperage * multiplier.coef[_windingIndex];
}

void button1Click() {
	if (modeWork.current >= MW_SETUP_START && modeWork.current <= MW_SETUP_STOP) {
		lcdUpdateScreen = true;
		if (modeWork.current == MW_SETUP_CONNECTION_TYPE) {
			setDigit.value = setDigit.value == CONNECTION_TYPE_STAR? CONNECTION_TYPE_TRIANGLE: CONNECTION_TYPE_STAR;
		} else if (modeWork.current >= MW_SETUP_GAIN_AMPERAGE && modeWork.current <= MW_SETUP_GAIN_VOLTAGE) {
			setDigit.value = setDigit.value == 5? 0: setDigit.value + 1;
		} else {
			setDigit.value = setDigit.value + setDigit.curMultiplier;
			if (setDigit.value >= 1000) {
				setDigit.value = 0;
			}
		}
		return;	
	}
	
	if (modeWork.current >= MW_SHOWING_START && modeWork.current <= MW_SHOWING_STOP) {
		modeWork.current = modeWork.current == MW_SHOWING_STOP? MW_SHOWING_START: modeWork.current + 1;
		return;
	}
	
	if (modeWork.current == MW_CONTROLL_MEASUREMENT) {
		modeWork.current = MW_SHOWING_START;
		settings.isReadyToWork = MM_WORK;
		initAvgVars();
		return;
	}
}

void button2Click() {
	if (modeWork.current == MW_NEED_SETUP) {
		lcdUpdateScreen = true;
		modeWork.current = MW_SETUP_START;
		setEditValue();
		settings.isSetupMode = true;
		return;
	}
		
	if (modeWork.current >= MW_SETUP_START && modeWork.current <= MW_SETUP_STOP) {
		lcdUpdateScreen = true;
		if (modeWork.current == MW_SETUP_CONNECTION_TYPE) {
			setDigit.value = setDigit.value == CONNECTION_TYPE_STAR? CONNECTION_TYPE_TRIANGLE: CONNECTION_TYPE_STAR; 
		} else if (modeWork.current >= MW_SETUP_GAIN_AMPERAGE && modeWork.current <= MW_SETUP_GAIN_VOLTAGE) {
			setDigit.value = setDigit.value == 0? 5: setDigit.value - 1;
		}else {
			setDigit.value = setDigit.value - setDigit.curMultiplier;
			if (setDigit.value < 0) {
				setDigit.value = 999.999f;
			}
		}
		return;
	}
	
	if (modeWork.current >= MW_SHOWING_START && modeWork.current <= MW_SHOWING_STOP) {
		modeWork.current = modeWork.current == MW_SHOWING_START? MW_SHOWING_STOP: modeWork.current - 1;
		return;
	}
	
	if (modeWork.current == MW_CONTROLL_MEASUREMENT) {
		modeWork.current = MW_SHOWING_START;
		settings.isReadyToWork = MM_STOP;
		return;
	}
}

void button1LongPressStart() {
	if (modeWork.current >= MW_SETUP_MULT_VOLTAGE_AB && modeWork.current <= MW_SETUP_MULT_AMPERAGE_AC) {
		// выбор множителя x1x10x100x0.1x0.01x0.001
		setDigit.curMultiplier = setDigit.curMultiplier >= 100.0f? 0.001f: setDigit.curMultiplier * 10.0f;
		lcdUpdateScreen = true;
		return;
	}
	
	if (modeWork.current >= MW_SETUP_IMPEDANCE_AB && modeWork.current <= MW_SETUP_IMPEDANCE_AC) {
		setDigit.value = getCurrentWindingImpedanceValue();
		lcdUpdateScreen = true;
		return;
	}
	
	if (modeWork.current >= MW_SHOWING_START && modeWork.current <= MW_SHOWING_STOP) {
		modeWork.current = MW_CONTROLL_MEASUREMENT;
		return;
	}
	
	if (modeWork.current == MW_CONTROLL_MEASUREMENT) {
		modeWork.current = MW_SHOWING_START;
	}
}

void button2LongPressStart() {
	if (icError.hasIC) {
		digitalWrite(BEEPER, LOW);
		icError.hasIC = false;
		return;
	}
	if (modeWork.current == MW_NEED_SETUP) {
		return;
	}
	if (modeWork.current == MW_SHOW_IMPEDANCE_CRITICLE_ERRORS) {
		for(byte i = 0; i < 3; i++) {
			icError.criticalLvlCount[i] = 0;
		}
		initAvgVars();
		modeWork.current = MW_SHOWING_START;
		lcdUpdateScreen = true;
		return;
	}
	
	
	if (settings.isSetupMode) {
		if (modeWork.current == MW_SETUP_STOP) {
			saveSettings();
			initMultiplierCoef();
			checkIsReadyToWork();
		} else {
			saveSettings();
			modeWork.current++;
			setEditValue();
		}
	} else {
		settings.isReadyToWork = MM_STOP;
		modeWork.current = MW_SETUP_START;
		setEditValue();
		settings.isSetupMode = true;
	}
}

void setEditValue() {
	lcdUpdateScreen = true;
	switch(modeWork.current) {
		case MW_SETUP_CONNECTION_TYPE:
			setDigit.value = settings.connectionType;
			break;
		case MW_SETUP_GAIN_AMPERAGE:
			setDigit.value = ads.currentAmperageGain;
			break;
		case MW_SETUP_GAIN_VOLTAGE:
			setDigit.value = ads.currentVoltageGain;
			break;			
		case MW_SETUP_IMPEDANCE_AB:
			setDigit.value = settings.impedance[0];
			break;
		case MW_SETUP_IMPEDANCE_BC:
			setDigit.value = settings.impedance[1];
			break;
		case MW_SETUP_IMPEDANCE_AC:
			setDigit.value = settings.impedance[2];
			break;
		case MW_SETUP_MULT_VOLTAGE_AB:
			setDigit.value = settings.multiplierVoltage[0];
			break;
		case MW_SETUP_MULT_VOLTAGE_BC:
			setDigit.value = settings.multiplierVoltage[1];
			break;
		case MW_SETUP_MULT_VOLTAGE_AC:
			setDigit.value = settings.multiplierVoltage[2];
			break;
		case MW_SETUP_MULT_AMPERAGE_AB:
			setDigit.value = settings.multiplierAmperage[0];
			break;
		case MW_SETUP_MULT_AMPERAGE_BC:
			setDigit.value = settings.multiplierAmperage[1];
			break;
		case MW_SETUP_MULT_AMPERAGE_AC:
			setDigit.value = settings.multiplierAmperage[2];
			break;
	}
	setDigit.curMultiplier = 1.0f;
}


void saveSettings() {
	switch(modeWork.current) {
		case MW_SETUP_CONNECTION_TYPE:
			settings.connectionType = setDigit.value;
			settings.criticleError = settings.connectionType == CONNECTION_TYPE_STAR? 2.5f: 5.0f;
			initImpedanceCriticalValue();
			eeprom_update_byte(&eeprom_connection_type, settings.connectionType);
			break;
		case MW_SETUP_GAIN_AMPERAGE:
			ads.currentAmperageGain = setDigit.value;
			initAdsAmperageGain();
			eeprom_update_byte(&eeprom_gain_amperage, ads.currentAmperageGain);
			break;
		case MW_SETUP_GAIN_VOLTAGE:
			ads.currentVoltageGain = setDigit.value;
			initAdsVoltageGain();
			eeprom_update_byte(&eeprom_gain_voltage, ads.currentVoltageGain);
			break;
		case MW_SETUP_IMPEDANCE_AB:
			settings.impedance[0] = setDigit.value;
			eeprom_update_float(&eeprom_impedance_ab, settings.impedance[0]);
			break;
		case MW_SETUP_IMPEDANCE_BC:
			settings.impedance[1] = setDigit.value;
			eeprom_update_float(&eeprom_impedance_bc, settings.impedance[1]);
			break;
		case MW_SETUP_IMPEDANCE_AC:
			settings.impedance[2] = setDigit.value;
			eeprom_update_float(&eeprom_impedance_ac, settings.impedance[2]);
			break;
		case MW_SETUP_MULT_VOLTAGE_AB:
			settings.multiplierVoltage[0] = setDigit.value;
			eeprom_update_float(&eeprom_voltage_mult_ab, settings.multiplierVoltage[0]);
			break;
		case MW_SETUP_MULT_VOLTAGE_BC:
			settings.multiplierVoltage[1] = setDigit.value;
			eeprom_update_float(&eeprom_voltage_mult_bc, settings.multiplierVoltage[1]);
			break;
		case MW_SETUP_MULT_VOLTAGE_AC:
			settings.multiplierVoltage[2] = setDigit.value;
			eeprom_update_float(&eeprom_voltage_mult_ac, settings.multiplierVoltage[2]);
			break;
		case MW_SETUP_MULT_AMPERAGE_AB:
			settings.multiplierAmperage[0] = setDigit.value;
			eeprom_update_float(&eeprom_amperage_mult_ab, settings.multiplierAmperage[0]);
			break;
		case MW_SETUP_MULT_AMPERAGE_BC:
			settings.multiplierAmperage[1] = setDigit.value;
			eeprom_update_float(&eeprom_amperage_mult_bc, settings.multiplierAmperage[1]);
			break;
		case MW_SETUP_MULT_AMPERAGE_AC:
			settings.multiplierAmperage[2] = setDigit.value;
			eeprom_update_float(&eeprom_amperage_mult_ac, settings.multiplierAmperage[2]);
			break;
	}
	
	if (modeWork.current == MW_SETUP_CONNECTION_TYPE || (modeWork.current >= MW_SETUP_IMPEDANCE_AB && modeWork.current <= MW_SETUP_IMPEDANCE_AC)) {
		initImpedanceCriticalValue();
	}

	if (modeWork.current >= MW_SETUP_MULT_VOLTAGE_AB && modeWork.current <= MW_SETUP_MULT_AMPERAGE_AC) {
		initMultiplierCoef();
	}
}

void setAdsGainByIndex(Adafruit_ADS1115* _ads, byte _index) {
	switch(_index) {
		case 0:
			adsVoltage.setGain(GAIN_TWOTHIRDS);
			break;
		case 1:
			adsVoltage.setGain(GAIN_ONE);
			break;
		case 2:
			adsVoltage.setGain(GAIN_TWO);
			break;
		case 3:
			adsVoltage.setGain(GAIN_FOUR);
			break;
		case 4:
			adsVoltage.setGain(GAIN_EIGHT);
			break;
		case 5:
			adsVoltage.setGain(GAIN_SIXTEEN);
			break;
	}
}


void initAdsVoltageGain() {
	setAdsGainByIndex(&adsVoltage, ads.currentVoltageGain);
	ads.voltageStep = ads.gainStep[ads.currentVoltageGain] / 1000.0;
}

void initAdsAmperageGain() {
	setAdsGainByIndex(&adsAmperage, ads.currentAmperageGain);
	ads.amperageStep = ads.gainStep[ads.currentAmperageGain] / 1000.0;
}