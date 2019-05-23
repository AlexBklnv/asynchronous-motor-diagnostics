#include <Arduino.h>
#include <Wire.h>
#include <avr/eeprom.h>
#include <Adafruit_ADS1015.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>	
/*
	todo list
	
	формирование сигнала на отключение + завершение диагностики
*/
/*
	Закон Ома по фасту
	I = V / R
	U = V * R
	R = U / I
*/

/************************************************************************
	Правило: у всех массивов отвечающие за обмотки индексы привязаны к следующей связке
	0 = AB
	1 = BC
	2 = AC													
************************************************************************/
#define BUTTON_1 A0						// Алиас пина кнопки 1
#define BUTTON_2 A1						// Алиас пина кнопки 2
#define BEEPER A2						// Алиас пина пищалки
#define RELAY A3						// Алиас пина реле
#define DEBUG 4							// Алиас пина дебага

#define CONNECTION_TYPE_STAR 0			// Идентификатор подключения по типу звезда
#define CONNECTION_TYPE_TRIANGLE 1		// Идентификатор подключения по типу треугольник

byte EEMEM eeprom_first_start = 0;							// AVRdude не заливает eeprom, потому идентификатор первого запуска для инициализации значений
byte EEMEM eeprom_connection_type = CONNECTION_TYPE_STAR;	// Тип подключени
byte EEMEM eeprom_gain_amperage = 0;						// Индекс усиления ацп по току
byte EEMEM eeprom_gain_voltage = 0;							// Индекс усиления ацп по напряжению
float EEMEM eeprom_impedance_ab = 0;						// Значение сопротивления обмотки AB
float EEMEM eeprom_impedance_bc = 0;						// Значение сопротивления обмотки BC
float EEMEM eeprom_impedance_ac = 0;						// Значение сопротивления обмотки AC
float EEMEM eeprom_voltage_mult_ab = 1;						// Значение множителя по напряжению для обмотки AB
float EEMEM eeprom_voltage_mult_bc = 1;						// Значение множителя по напряжению для обмотки BC					
float EEMEM eeprom_voltage_mult_ac = 1;						// Значение множителя по напряжению для обмотки AC	
float EEMEM eeprom_amperage_mult_ab = 1;					// Значение множителя по току для обмотки AB				
float EEMEM eeprom_amperage_mult_bc = 1;					// Значение множителя по току для обмотки AB
float EEMEM eeprom_amperage_mult_ac = 1;					// Значение множителя по току для обмотки AB

#define MM_STOP false	// Алиас режима ожидания
#define MM_WORK true	// Алиас режима измерения

// Режимы работы
// Режим настроек
#define MW_NEED_SETUP 0										// Первый запуск
#define MW_SETUP_CONNECTION_TYPE 1							// Установка типа соединения обмоток
#define MW_SETUP_GAIN_VOLTAGE 2								// Установка степени усиления АЦП по напряжению
#define MW_SETUP_GAIN_AMPERAGE 3							// Установка степени усиления АЦП по току
#define MW_SETUP_MULT_VOLTAGE_AB 4							// Установка значения множителя напряжения для обмотки AB
#define MW_SETUP_MULT_VOLTAGE_BC 5							// Установка значения множителя напряжения для обмотки BC
#define MW_SETUP_MULT_VOLTAGE_AC 6							// Установка значения множителя напряжения для обмотки AC
#define MW_SETUP_MULT_AMPERAGE_AB 7							// Установка значения множителя тока для обмотки AB
#define MW_SETUP_MULT_AMPERAGE_BC 8							// Установка значения множителя тока для обмотки BC
#define MW_SETUP_MULT_AMPERAGE_AC 9							// Установка значения множителя тока для обмотки AC
#define MW_SETUP_IMPEDANCE 10							// Установка значения сопротивления для обмотки AB

#define MW_SETUP_START MW_SETUP_CONNECTION_TYPE				// Алиас для стартового пункта меню настроек
#define MW_SETUP_STOP MW_SETUP_IMPEDANCE					// Алиас для последнего пункта меню настроек

// Showing characteristics
#define MW_SHOW_ERRORS_COUNTERS 13				// Отображения количества выхода за предел критического значениия { счетчик ошибок по току AB | счетчик ошибок по току BC | счетчик ошибок по току AC }
#define MW_SHOW_ERRORS 14						// Оторажения в процентах степени отклоенения текущего тока от иделаьного { ошибка по току AB | ошибка по току BC | ошибка по току AC }
#define MW_SHOW_AMPERAGE 15					// Отображение всех токов
#define MW_SHOW_AMPERAGE_AB 16					// Отображение силы тока обмотки AB { M - измеренный | P - идеальный}
#define MW_SHOW_AMPERAGE_BC 17					// Отображение силы тока обмотки  BC { M - измеренный | P - идеальный}
#define MW_SHOW_AMPERAGE_AC 18					// Отображение силы тока обмотки  AC { M - измеренный | P - идеальный}
#define MW_SHOW_WINDING_CHARS_AB 19				// Отображение характеристик обмотки AB { напряжение | степень огшибки по току | среднее значение силы тока за N измерений }
#define MW_SHOW_WINDING_CHARS_BC 20				// Отображение характеристик обмотки BC { напряжение | степень огшибки по току | среднее значение силы тока за N измерений }
#define MW_SHOW_WINDING_CHARS_AC 21				// Отображение характеристик обмотки AC { напряжение | степень огшибки по току | среднее значение силы тока за N измерений }

#define MW_SHOWING_START MW_SHOW_ERRORS_COUNTERS	// Алиас для стартового пункта отображения данных
#define MW_SHOWING_STOP MW_SHOW_WINDING_CHARS_AC 	// Алиас для последнего пункта отображения данных

#define MW_CONTROLL_MEASUREMENT	30					// Дополнительный пункт меню, выбор о старте измерений


/*	
	Таблица усиления АЦП
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

// Объекты работы с АЦП модулями
Adafruit_ADS1115 adsVoltage(0x48);			
Adafruit_ADS1115 adsAmperage(0x49);

/*
	Сборка связанная с вычислением релаьного значения сигнала с АЦП за бит
	gainStep - поиндексно вынесены цены за бит для выборки в настроках
	voltageStep - множитель по напрядению 
	amperageStep - множитель по току
*/
struct Ads1115 {
	float gainStep[6] = {0.1875, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125};
	float voltageStep = 0.1875 / 1000.0;
	float amperageStep = 0.1875 / 1000.0;
};

/*
	Характеристики снятые с АЦП
	voltage - значение напряжения за measurementsCount количества измерений
	measuredAmperage - значение силы тока за measurementsCount количества измерений
	perfectAmperage - идеальное значения силы тока, вычисляемое перед использованием
	sumVoltage - сюда заливаем для усреднения значения напряжений
	sumMeasuredAmperage - сюда заливаем для усреднения значения силы тока
	measurementsCount - количество измерений перед усреднением
	currentMeasurement - номер текущего измерения
*/
struct AdsChars {
	float voltage[3] = {0, 0, 0};
	float measuredAmperage[3] = {0, 0, 0};
	float perfectAmperage[3] = {0, 0, 0};
	float sumVoltage[3] = {0, 0, 0};
	float sumMeasuredAmperage[3] = {0, 0, 0};
	byte measurementsCount = 60;
	byte currentMeasurement = 1;
};	

/*
	Настройки
	isReadyToWork - флаг, готовность к работе комплекса
	connectionType - тип подключения
	criticleError - остаток от сравнения по сопротивлениям, сейчас не используется
	impedance - значение сопротивлений
	isSetupMode - флаг, нахождение в режиме настроек
	currentAmperageGain - индекс усиления АЦП по току
	currentVoltageGain - индекс усиления АЦП по напряжению
	multiplierVoltage - множитель по напряжению для вычисления реального значения
	multiplierAmperage - множитель по току для вычисления реального значения
*/
struct Settings {
	bool isReadyToWork = false;
	bool connectionType = CONNECTION_TYPE_STAR;
	float criticleError = 2.5;
	float impedance[3] = {0, 0, 0};
	// true - settings | false - characteristics
	bool isSetupMode;
	byte currentAmperageGain = 0;
	byte currentVoltageGain = 0;
	float multiplierVoltage[3] = {1, 1, 1};
	float multiplierAmperage[3] = {1, 1, 1};
	bool isDebugMode = false;
};

/*
	Структура отклоения релаьного от иделаьного значения 
	curLvl - Уровень ошибки текущий за определенное количество измерений
	criticalLvlCount - Количество выходов за пределы допуска
	hasIC - флаг, наличие межвиткового замыкания
	criticalSingle - отклонение на этой величины единичной обмотки критично
	criticalFull - отклонение общей ошибки более чем на это число критично
*/
struct Error {
	float curLvl[3] = {0, 0, 0};
	int criticalLvlCount[3] = {0, 0, 0};
	bool hasIC = false;
	unsigned long detectStamp = 0;
	float criticalSingle = 0.85;
	float criticalFull = 1;
};

/*
	Вспомогательная структура для настроек
	value - текуцщее изменяемое значение
	curMultiplier - для установки множителей, ценаза клик кнопки x0.001-x100.0
*/
struct SetDigit {
	float value;
	float curMultiplier;
};

/*
	Режим работы
	current - текущий
	prev - предыдущий
*/
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

bool lcdUpdateScreen = false;						// Флаг, необходимость обновить экран
unsigned long serialUpdateStamp = 0;				// Пора ли отправлять данные в Serial порт
	
void initAvgVars();
void getAdsParams();
void saveSettings();
void button2Click();
void button1Click();
void setEditValue();
void processAsMode();
void displayAsMode();
void taskMeasurement();
void showAmperageChars(byte);
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
float getICLevelByWinding(byte);
void calculateRealAdsParams();
bool isFullErrorExists(float*,float*);
void lcdPrintCriticalLvl(byte _num);
void showWindingCharsValues(byte _num);
void lcdPrinRoundedCurErrorLevel(byte _num);
void setAdsGainByIndex(Adafruit_ADS1115*, byte);
void lcdClearCell(byte col, byte row, byte rowLength);
void setupImpedance();
bool isSingleErrorExists(float, float, byte);

void setup() {
	Serial.begin(9600);
	Serial.println(F("Initialization..."));
	
	// Инициализация АЦП
	adsVoltage.setGain(GAIN_TWOTHIRDS);
	adsVoltage.begin();
	adsAmperage.setGain(GAIN_TWOTHIRDS);
	adsAmperage.begin();
	// Инициализация LCD модуля
	lcd.init();
	lcd.backlight();
	lcd.clear();
	Serial.println(F("LCD inited!"));
	
	// Инициализая кнопок
	button1Callback.attachClick(button1Click);								// Установка callback функции на короткий клик
	button1Callback.attachLongPressStart(button1LongPressStart);			// Установка callback функции на длинный клик
	button1Callback.setDebounceTicks(30);									// Время до идентификации клика в мс для нивилирования дребезга контактов
	button1Callback.setClickTicks(200);										// Количество мс нажатия для того чтобы клик был идентифицирован
	button1Callback.setPressTicks(1000);									// Количество мс нажатия для идентификаци длинного клика
	Serial.println(F("Button 1 inited!"));
	
	button2Callback.attachClick(button2Click);
	button2Callback.attachLongPressStart(button2LongPressStart);
	button2Callback.setDebounceTicks(30);
	button2Callback.setClickTicks(200);
	button2Callback.setPressTicks(1000);
	Serial.println(F("Button 2 inited!"));
	
	// Инициализация старт-пакета данных
	if (eeprom_read_byte(&eeprom_first_start) != 100) {
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
		eeprom_update_byte(&eeprom_first_start, 100);
		Serial.println(F("EEPROM first start writed!"));
	}
	
	// Теперь старт данные получаем из памяти при старте
	settings.currentAmperageGain = eeprom_read_byte(&eeprom_gain_amperage);
	settings.currentVoltageGain = eeprom_read_byte(&eeprom_gain_voltage);
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
	Serial.println(F("EEPROM values:"));
	Serial.print(F("gain_amperage: ")); Serial.println(settings.currentAmperageGain);
	Serial.print(F("gain_voltage: ")); Serial.println(settings.currentVoltageGain);
	Serial.print(F("connection_type: ")); Serial.println(settings.connectionType);
	Serial.print(F("impedance_ab: ")); Serial.println(settings.impedance[0], 10);
	Serial.print(F("impedance_bc: ")); Serial.println(settings.impedance[1], 10);
	Serial.print(F("impedance_ac: ")); Serial.println(settings.impedance[2], 10);
	Serial.print(F("voltage_mult_ab: ")); Serial.println(settings.multiplierVoltage[0], 3);
	Serial.print(F("voltage_mult_bc: ")); Serial.println(settings.multiplierVoltage[1], 3);
	Serial.print(F("voltage_mult_ac: ")); Serial.println(settings.multiplierVoltage[2], 3);
	Serial.print(F("amperage_mult_ab: ")); Serial.println(settings.multiplierAmperage[0], 3);
	Serial.print(F("amperage_mult_bc: ")); Serial.println(settings.multiplierAmperage[1], 3);
	Serial.print(F("amperage_mult_ac: ")); Serial.println(settings.multiplierAmperage[2], 3);
	
	initAdsVoltageGain();
	initAdsAmperageGain();
	
	settings.criticleError = settings.connectionType == CONNECTION_TYPE_STAR? 2.5f: 5.0f;

	checkIsReadyToWork();
	
	// Инициализируем пищалку
	pinMode(RELAY, OUTPUT);
	pinMode(BEEPER, OUTPUT);
	pinMode(DEBUG, INPUT);
	digitalWrite(RELAY, LOW);
	digitalWrite(BEEPER, LOW);
	
	if (digitalRead(DEBUG) == HIGH) {
		settings.isDebugMode = true;
		lcd.clear();
		lcd.print(F("Debug mode"));
		delay(1000);
	}
	
	modeWork.prev = modeWork.current;
	initAvgVars();
	Serial.println(F("Completed!"));
	Serial.println(F("Stand by..."));
	displayStaticAsMode();
	displayAsMode();
}

/*
	Проверка на первый старт и необходимость настреок перед стартом работы
*/
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
		Serial.println(F("Need setup params"));
	}
}

void loop() {
	// Следим за кнопками
	button1Callback.tick();
	button2Callback.tick();
	// Отображаем данные на экран
	displayAsMode();
	// Если в режиме измерений
	
	if(settings.isReadyToWork == MM_WORK) {
		// получаем параметры
		getAdsParams();
		// и продолжаем получать пока не накопим 50 измерений
		if (adsChars.currentMeasurement < adsChars.measurementsCount) {
			adsChars.currentMeasurement++;
		} else {
			// после накопления measurementsCount обновляем экран и смотрим на наличия межветковых замыканий
			lcdUpdateScreen = true;
			bool canSerial = millis() - serialUpdateStamp > 1000? true: false;
			for (byte i = 0; i < 3; i++) {
				
				// вычисляем реальные значения напряжения, тока и идеального тока от измеренного напряжения и известного сопротивления
				adsChars.voltage[i] = adsChars.sumVoltage[i] / adsChars.measurementsCount * ads.voltageStep * settings.multiplierVoltage[i];
				adsChars.measuredAmperage[i] = adsChars.sumMeasuredAmperage[i] / adsChars.measurementsCount * ads.amperageStep * settings.multiplierAmperage[i];
				adsChars.perfectAmperage[i] = adsChars.voltage[i] / settings.impedance[i];
				icError.curLvl[i] = getICLevelByWinding(i);

				if (canSerial) {
					Serial.print(F("Winding |")); Serial.print((i + 1)); Serial.println(F("|"));
					Serial.print(F("measured voltage = "));  Serial.println(adsChars.voltage[i], 6);
					Serial.print(F("measured amperage = "));  Serial.println(adsChars.measuredAmperage[i], 6);
					Serial.print(F("perfect amperage = ")); Serial.println(adsChars.perfectAmperage[i], 6);
					Serial.print(F("error = ")); Serial.println(icError.curLvl[i]);
					if (i != 2) {
						Serial.println(F("---"));
					}
				}
			}
			if (canSerial) {
				Serial.println(F("---end---"));
				serialUpdateStamp = millis();
			}
			initAvgVars();
	
			
			if (isFullErrorExists(adsChars.measuredAmperage, adsChars.voltage)) {
				if (icError.detectStamp == 0) {
					Serial.println(F("Probably error exists"));
					icError.detectStamp = millis();
				}
				if (millis() - icError.detectStamp >= 10000) {
					if (!settings.isDebugMode) {
						settings.isReadyToWork = MM_STOP;
						digitalWrite(RELAY, HIGH);
					}
					
					Serial.println(F("WARNING!!! Measurement Stoped IC EXISTS!"));
					digitalWrite(BEEPER, HIGH);
					icError.hasIC = true;
				}
			} else {
				if (icError.detectStamp != 0) {
					Serial.println(F("ha-ha, it's joke. There are not exists critical error"));
				}
				icError.detectStamp = 0;
			}
		}
	} 
}

/**
 * Получение параметров с АЦП
 */
void getAdsParams() {
	// Временные данные по напряжению, току и иделаьному току
	float measuredVoltage[3] = {0, 0, 0};
	float measuredAmperage[3] = {0, 0, 0};
	float perfectAmperage[3] = {0, 0, 0};
		
	// получение данных с ацп для кжадой обмотки
	for (byte i = 0; i < 3; i++) {
		measuredVoltage[i] = adsVoltage.readADC_SingleEnded(i);
		measuredAmperage[i] = adsAmperage.readADC_SingleEnded(i);
	}

	
	for (byte i = 0; i < 3; i++) {
		// увеличиваем данные для усреднения
		adsChars.sumVoltage[i] += measuredVoltage[i];
		adsChars.sumMeasuredAmperage[i] += measuredAmperage[i];
		// вычисляем текущие реальные значения
		measuredVoltage[i] *= ads.voltageStep * settings.multiplierVoltage[i];
		measuredAmperage[i] *= ads.amperageStep * settings.multiplierAmperage[i];
		
		// прогоняем для проверки на выходы за пределы допуска и увеличиваем счетчик, ели это случилось
		if (icError.criticalLvlCount[i] < 1000 && isSingleErrorExists(measuredAmperage[i], measuredVoltage[i], i)) {
			icError.criticalLvlCount[i]++;
		}
	}
}

/*
	Обнуление переменных - средние значения характеристик и сбрасываем счетчик измерений
*/
void initAvgVars() {
	for (byte i = 0; i < 3; i++) {
		adsChars.sumVoltage[i] = 0;
		adsChars.sumMeasuredAmperage[i] = 0;
	}
	adsChars.currentMeasurement = 1;
}

/*
	Получения степени отклонения силы тока идеального от измеренного по обмотке
	_amperage - измеренное значение тока
	_num - номер обмотки
*/
float getICLevelByWinding(byte _num) {
	if (adsChars.perfectAmperage[_num] >= adsChars.measuredAmperage[_num]) {
		return 0;
	}
	return abs(adsChars.measuredAmperage[_num] - adsChars.perfectAmperage[_num]) * 100.0 / ( adsChars.perfectAmperage[_num] + icError.criticalSingle);
}

/*
 * Проверка наличия локального отклоеннеия.
 * Ошибка проверяется только при положительных разностях
 * _amperage - массив измеренных токов
 * _voltage - массив измеренных напряжений
 * index - индекс
*/
bool isSingleErrorExists(float _amperage, float _voltage, byte index) {
	// высчитываем идеальный ток и отнимаем измеренный
	float divAmperage = _amperage - (_voltage / settings.impedance[index]);
	if (divAmperage <= 0) {
		return false;
	}
	return divAmperage >= icError.criticalSingle;
}

/*
 * Проверка наличия отклоеннеия.
 * Ошибка проверяется только при положительных разностях
 * _amperage - массив измеренных токов
 * _voltage - массив измеренных напряжений
*/
bool isFullErrorExists(float* _amperage, float* _voltage) {
	float divAmperage[3] = {0, 0, 0};
	// 100 - магическое число и ничего не значит, прсото для удобства 
	byte sign = 100;
	
	// получаем разницу идеального от измеренного тока и изменяем значения счетчика знака
	for(byte i = 0; i < 3; i++) {
		// на ходу вычисляем значения идеального тока
		adsChars.perfectAmperage[i] = _voltage[i] / settings.impedance[i];
		divAmperage[i] = adsChars.perfectAmperage[i] - _amperage[i];
		if (divAmperage[i] > 0) {
			sign++;
		} else {
			sign--;
		}
	}
	
	// если знак = -97, значит все значения измеренных токов меньше идеальных и замыкания нет
	if (sign == -97) {
		return false;
	}
	// По формуле вычисляем глоабльную ошибку по токам |AB - BC| + |BC - AC| + |AC - AB| 
	float _error = abs(divAmperage[0] - divAmperage[1]) + abs(divAmperage[1] - divAmperage[2]) + abs(divAmperage[2] - divAmperage[0]);

	// Крит общей ошибки отклонение в 1
	return _error >= icError.criticalFull;
}

/*
	Оторажаем возможные варианты усиления
*/
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
			lcd.print(F("2.048 0.0625"));
			break;
		case 3:
			lcd.print(F("1.024 0.03125"));
			break;
		case 4:
			lcd.print(F("0.512 0.015625"));
			break;
		case 5:
			lcd.print(F("0.256 0.0078125"));
			break;
	}
}

/*
	Отображение динамиеских данных в зависимости от режима
*/
void displayAsMode() {
	// Если мы изменили режим отображения, то необходимо обновиь статику и динамику
	if (modeWork.current != modeWork.prev) {
		modeWork.prev = modeWork.current;
		lcdUpdateScreen = true;
		displayStaticAsMode();
	}
	if (!lcdUpdateScreen) {
		return;
	} 
	lcdUpdateScreen = false;
	// обновляем динамику
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
		case MW_SETUP_IMPEDANCE:
			lcd.print(F("R1="));
			lcd.print(settings.impedance[0], 1);
			lcd.setCursor(8, 0);
			lcd.print(F("R2="));
			lcdClearCell(11, 0, 5);
			lcd.print(settings.impedance[1], 1);
			lcd.setCursor(0, 1);
			lcd.print(F("R3="));
			lcdClearCell(3, 1, 5);
			lcd.print(settings.impedance[2], 1);
			break;
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
		case MW_SHOW_ERRORS_COUNTERS:
			lcdClearCell(11, 0, 4);
			lcdPrintCriticalLvl(0);
			lcdClearCell(3, 1, 4);
			lcdPrintCriticalLvl(1);
			lcdClearCell(11, 1, 4);
			lcdPrintCriticalLvl(2);
			break;
		case MW_SHOW_ERRORS:
			lcdClearCell(3, 0, 5);
			lcdPrinRoundedCurErrorLevel(0);
			lcdClearCell(11, 0, 5);
			lcdPrinRoundedCurErrorLevel(1);
			lcdClearCell(3, 1, 5);
			lcdPrinRoundedCurErrorLevel(2);
			break;
		case MW_SHOW_AMPERAGE:
			lcd.clear();
			lcd.print(F("P "));
			lcd.print(adsChars.perfectAmperage[0], 2);
			lcd.print(F("#"));
			lcd.print(adsChars.perfectAmperage[1], 2);
			lcd.print(F("#"));
			lcd.print(adsChars.perfectAmperage[2], 2);
	
			lcd.setCursor(0, 1);
			lcd.print(F("M "));
			lcd.print(adsChars.measuredAmperage[0], 2);
			lcd.print(F("#"));
			lcd.print(adsChars.measuredAmperage[1], 2);
			lcd.print(F("#"));
			lcd.print(adsChars.measuredAmperage[2], 2);
			break;
		case MW_SHOW_AMPERAGE_AB:
			showAmperageChars(0);
			break;
		case MW_SHOW_AMPERAGE_BC:
			showAmperageChars(1);
			break;
		case MW_SHOW_AMPERAGE_AC:
			showAmperageChars(2);
			break;
		case MW_SHOW_WINDING_CHARS_AB:
			showWindingCharsValues(0);
			break;
		case MW_SHOW_WINDING_CHARS_BC:
			showWindingCharsValues(1);
			break;
		case MW_SHOW_WINDING_CHARS_AC:
			showWindingCharsValues(2);
			break;
	}
}

/*
	Отображение округленных значений ошибки в зависимости от обмотки 
	_num - номер обмотки
*/
void lcdPrinRoundedCurErrorLevel(byte _num) {
	if (icError.curLvl[_num] < 1000) {
		lcd.print(round(icError.curLvl[_num]));
	} else {
		lcd.print(999);
		lcd.print(F("+"));
	}
}

/*
	Отображение счетчиков выхода за пределы допуска ошибки
	_num - номер обмотки
*/
void lcdPrintCriticalLvl(byte _num) {
	if (icError.criticalLvlCount[_num] < 1000) {
		lcd.print(icError.criticalLvlCount[_num]);
	} else {
		lcd.print(999);
		lcd.print(F("+"));
	}
}

/*
	Отображение статики в ависимости от режима отображения
*/
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
		case MW_SHOW_ERRORS_COUNTERS:
			lcd.print(F("ECount"));
			lcd.setCursor(8, 0);
			lcd.print(F("AB="));
			lcd.setCursor(0, 1);
			lcd.print(F("BC="));
			lcd.setCursor(8, 1);
			lcd.print(F("AC="));
			break;
		case MW_SHOW_ERRORS:
			lcd.print(F("I1="));
			lcd.setCursor(8, 0);
			lcd.print(F("I2="));
			lcd.setCursor(0, 1);
			lcd.print(F("I3="));
			lcd.setCursor(14, 1);
			lcd.print(F("e%"));
			break;
		case MW_SHOW_AMPERAGE_AB:
			lcd.print(F("AB D"));
			showStaticAmperage();
			break;
		case MW_SHOW_AMPERAGE_BC:
			lcd.print(F("BC D"));
			showStaticAmperage();
			break;
		case MW_SHOW_AMPERAGE_AC:
			lcd.print(F("AC D"));
			showStaticAmperage();
			break;
		case MW_SHOW_WINDING_CHARS_AB:
			lcd.print(F("AB"));
			showStaticWindingChars();
			break;
		case MW_SHOW_WINDING_CHARS_BC:
			lcd.print(F("BC"));
			showStaticWindingChars();
			break;
		case MW_SHOW_WINDING_CHARS_AC:
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

/*
	Отображение идеального и измеренного тока
	_num - номер обмотки
*/
void showAmperageChars(byte _num) {
	lcdClearCell(9, 0, 7);
	lcd.print(adsChars.perfectAmperage[_num], 5);
	
	lcdClearCell(0, 1, 6);
	float _diff = abs(adsChars.measuredAmperage[_num] - adsChars.perfectAmperage[_num]);
	lcd.print(_diff, 4);
	
	lcdClearCell(9, 1, 7);
	lcd.print(adsChars.measuredAmperage[_num], 5);
}

/*
	Отображение дополнительной статики для режима отображения по току
*/
void showStaticAmperage() {
	lcd.setCursor(7, 0);
	lcd.print(F("P"));
	lcd.setCursor(7, 1);
	lcd.print(F("M"));
}

/*
	Отображение характеристик по обмоткам
	_num - номер обмотки
*/
void showWindingCharsValues(byte _num) {
	lcdClearCell(5, 0, 7);
	byte roundedSign = 2;
	lcd.print(adsChars.measuredAmperage[_num], 5);
	
	lcdClearCell(2, 1, 8);
	if (adsChars.voltage[_num] < 100) {
		roundedSign = 2;
	} else if (adsChars.voltage[_num] < 1000) {
		roundedSign = 1;
	} else {
		roundedSign = 0;
	}
	if (roundedSign > 0) {
		lcd.print(adsChars.voltage[_num], roundedSign);
	} else {
		lcd.print(999);
		lcd.print(F("+"));
	}
	
	lcdClearCell(10, 1, 5);
	lcd.print(icError.curLvl[_num], 1);
	lcd.print(F("%"));
}



/*
	Отображение статики по обмоткам
	MP значит measured|perfect
*/
void showStaticWindingChars() {
	lcd.setCursor(3, 0);
	lcd.print(F("I="));
	lcd.setCursor(0, 1);
	lcd.print(F("V="));
	lcd.setCursor(8, 1);
	lcd.print(F("E="));
}

/*
	Очистка определенного блока экрана с установкой на этом месте курсора
	col - номер колонки
	row - номер строки
	rowLength - количество ячеек для очистки в строке
*/
void lcdClearCell(byte col, byte row, byte rowLength) {                
	lcd.setCursor(col, row);                                        
	for (byte i = 0; i < rowLength; i++) {                             
		lcd.print(F(" "));
	}
	lcd.setCursor(col, row);                                          
}

/*
	Получение текущего значения сопротивления обмотки от измеренных значений напряжения и силы тока
*/
void setupImpedance() {
	float _voltage[3] = {0, 0, 0};
	float _amperage[3] = {0, 0, 0};
	float _sumImpedance[3] = {0, 0, 0};
	float _sumVoltage[3] = {0, 0, 0};
	float _sumAamperage[3] = {0, 0, 0};
	byte index = 1;
	
	while(index < adsChars.measurementsCount) {
		for(byte i = 0; i < 3; i++) {
			_voltage[i] = adsVoltage.readADC_SingleEnded(i);
			_amperage[i] = adsAmperage.readADC_SingleEnded(i);
		}
		
		for(byte i = 0; i < 3; i++) {
			_sumVoltage[i] += _voltage[i];
			_sumAamperage[i] += _amperage[i];
		}
		index++;
	}
	
	Serial.println(F("Impedance setup"));
				
	for(byte i = 0; i < 3; i++) {
		Serial.println(i);
		_voltage[i] = _sumVoltage[i] / adsChars.measurementsCount * ads.voltageStep * settings.multiplierVoltage[i];
		_amperage[i] = _sumAamperage[i] / adsChars.measurementsCount * ads.amperageStep * settings.multiplierAmperage[i];
		settings.impedance[i] = _voltage[i] / _amperage[i];
		Serial.print(F("V=")); Serial.println(_voltage[i], 5);
		Serial.print(F("A=")); Serial.println(_amperage[i], 5);
		Serial.print(F("R=")); Serial.println(settings.impedance[i], 5);
		if (i == 2) {
			Serial.println(F("---end---"));
		} else {
			Serial.println(F("------"));
		}
	}
}

/*
	Обработчика короткого клика первой клавиши
*/
void button1Click() {
	// Если мы в настройках сопротивления то ничего не делать
	if (modeWork.current == MW_SETUP_IMPEDANCE) {
		return;
	}
	// Если мы в любых других настройках
	if (modeWork.current >= MW_SETUP_START && modeWork.current <= MW_SETUP_STOP) {
		lcdUpdateScreen = true;
		if (modeWork.current == MW_SETUP_CONNECTION_TYPE) {
			// Режим выбора подключения обмотки - изменяем его
			setDigit.value = setDigit.value == CONNECTION_TYPE_STAR? CONNECTION_TYPE_TRIANGLE: CONNECTION_TYPE_STAR;
		} else if (modeWork.current >= MW_SETUP_GAIN_AMPERAGE && modeWork.current <= MW_SETUP_GAIN_VOLTAGE) {
			// Режим выбора усиления АЦП - меняем в пределах от 0-5 в большую сторону
			setDigit.value = setDigit.value == 5? 0: setDigit.value + 1;
		} else {
			// Любой другой режим увеличиваем значение на величину curMultiplier
			setDigit.value = setDigit.value + setDigit.curMultiplier;
			if (setDigit.value >= 1000) {
				setDigit.value = 0;
			}
		}
		return;	
	}
	
	// В режиме отображения данных листаем вперед меню
	if (modeWork.current >= MW_SHOWING_START && modeWork.current <= MW_SHOWING_STOP) {
		modeWork.current = modeWork.current == MW_SHOWING_STOP? MW_SHOWING_START: modeWork.current + 1;
		return;
	}
	
	// Если вопрос о начале измерений, то клик его подтверждает
	if (modeWork.current == MW_CONTROLL_MEASUREMENT) {
		modeWork.current = MW_SHOWING_START;
		settings.isReadyToWork = MM_WORK;
		icError.hasIC = false;
		digitalWrite(RELAY, LOW);
		
		Serial.println(F("**Measurement Started!"));
		initAvgVars();
		return;
	}
}

/*
	Обработчика короткого клика второй клавиши
*/
void button2Click() {
	// Если мы в настройках сопротивления то обнуляем значения сопротивлений
	if (modeWork.current == MW_SETUP_IMPEDANCE) {
		settings.impedance[0] = 0;
		settings.impedance[1] = 0;
		settings.impedance[2] = 0;
		lcdUpdateScreen = true;
		return;
	}
	
	// В режиме предупреждения необходимости настроек - переводим в режим настроек
	if (modeWork.current == MW_NEED_SETUP) {
		lcdUpdateScreen = true;
		modeWork.current = MW_SETUP_START;
		setEditValue();
		settings.isSetupMode = true;
		return;
	}
		
	// Если режим настроек
	if (modeWork.current >= MW_SETUP_START && modeWork.current <= MW_SETUP_STOP) {
		lcdUpdateScreen = true;
		if (modeWork.current == MW_SETUP_CONNECTION_TYPE) {
			// режим выбора подключения - свапаем режим
			setDigit.value = setDigit.value == CONNECTION_TYPE_STAR? CONNECTION_TYPE_TRIANGLE: CONNECTION_TYPE_STAR; 
		} else if (modeWork.current >= MW_SETUP_GAIN_AMPERAGE && modeWork.current <= MW_SETUP_GAIN_VOLTAGE) {
			// Режим выбора усиления АЦП - меняем в пределах от 0-5 в меньшую сторону
			setDigit.value = setDigit.value > 0?  setDigit.value - 1: 0;
		}else {
			// Любой другой режим уменьшаем значение на величину curMultiplier
			setDigit.value = setDigit.value - setDigit.curMultiplier;
			if (setDigit.value < 0) {
				setDigit.value = 999.999f;
			}
		}
		return;
	}
	
	// В режиме отображения данных листаем назад меню
	if (modeWork.current >= MW_SHOWING_START && modeWork.current <= MW_SHOWING_STOP) {
		modeWork.current = modeWork.current == MW_SHOWING_START? MW_SHOWING_STOP: modeWork.current - 1;
		return;
	}
	
	// Если вопрос о начале измерений, то клик его отклоняет
	if (modeWork.current == MW_CONTROLL_MEASUREMENT) {
		modeWork.current = MW_SHOWING_START;
		settings.isReadyToWork = MM_STOP;
		return;
	}
}

/*
	Обработчик длиннкого клика первой клавиши
*/
void button1LongPressStart() {
	// выбор множителя x1x10x100x0.1x0.01x0.001 в режимах установки множителя
	if (modeWork.current >= MW_SETUP_MULT_VOLTAGE_AB && modeWork.current <= MW_SETUP_MULT_AMPERAGE_AC) {
		setDigit.curMultiplier = setDigit.curMultiplier >= 100.0f? 0.001f: setDigit.curMultiplier * 10.0f;
		lcdUpdateScreen = true;
		return;
	}
	
	// получение текущих значений сопротивления в режимах установки импеданса
	if (modeWork.current == MW_SETUP_IMPEDANCE) {
		lcd.clear();
		lcd.print(F("Impedance setup"));
		setupImpedance();
		lcd.clear();
		modeWork.prev = 0;
		return;
	}
	
	// В режимах просмотра данных, длинный клик переводит в режим вопроса о продолжении измерений
	if (modeWork.current >= MW_SHOWING_START && modeWork.current <= MW_SHOWING_STOP) {
		modeWork.current = MW_CONTROLL_MEASUREMENT;
		return;
	}
	
	// В ржеиме выбора о старте измерений - отклоенние вопроса
	if (modeWork.current == MW_CONTROLL_MEASUREMENT) {
		modeWork.current = MW_SHOWING_START;
	}
}

/*
	Обработчик длинного клика второй клавиши
*/
void button2LongPressStart() {
	// При наличии замыкания отключает пищалку
	if (icError.hasIC) {
		digitalWrite(BEEPER, LOW);
		icError.hasIC = false;
		icError.detectStamp = false;
		return;
	}
	
	// В режиме предупреждения о необходимости натсроек ничего не делает
	if (modeWork.current == MW_NEED_SETUP) {
		return;
	}
	
	// В режиме отображения количества выхода за пределы измерений обнуляет их и средние значения
	if (modeWork.current == MW_SHOW_ERRORS_COUNTERS) {
		for(byte i = 0; i < 3; i++) {
			icError.criticalLvlCount[i] = 0;
		}
		initAvgVars();
		modeWork.current = MW_SHOWING_START;
		lcdUpdateScreen = true;
		return;
	}
	
	// Перевод в режим настроек и выход из него
	if (settings.isSetupMode) {
		if (modeWork.current == MW_SETUP_STOP) {
			saveSettings();
			checkIsReadyToWork();
		} else {
			saveSettings();
			modeWork.current++;
			setEditValue();
		}
	} else {
		Serial.println(F("**Measurement Stoped!"));
		settings.isReadyToWork = MM_STOP;
		modeWork.current = MW_SETUP_START;
		setEditValue();
		settings.isSetupMode = true;
	}
}

/*
	Получение значений настройки в настроечную переменную для изменения во время настроек
*/
void setEditValue() {
	lcdUpdateScreen = true;
	switch(modeWork.current) {
		case MW_SETUP_CONNECTION_TYPE:
			setDigit.value = settings.connectionType;
			break;
		case MW_SETUP_GAIN_AMPERAGE:
			setDigit.value = settings.currentAmperageGain;
			break;
		case MW_SETUP_GAIN_VOLTAGE:
			setDigit.value = settings.currentVoltageGain;
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

/*
	Сохранение настроек при сменах режима
*/
void saveSettings() {
	switch(modeWork.current) {
		case MW_SETUP_CONNECTION_TYPE:
			settings.connectionType = setDigit.value;
			settings.criticleError = settings.connectionType == CONNECTION_TYPE_STAR? 2.5f: 5.0f;
			eeprom_update_byte(&eeprom_connection_type, settings.connectionType);
			break;
		case MW_SETUP_GAIN_AMPERAGE:
			settings.currentAmperageGain = setDigit.value;
			initAdsAmperageGain();
			eeprom_update_byte(&eeprom_gain_amperage, settings.currentAmperageGain);
			break;
		case MW_SETUP_GAIN_VOLTAGE:
			settings.currentVoltageGain = setDigit.value;
			initAdsVoltageGain();
			eeprom_update_byte(&eeprom_gain_voltage, settings.currentVoltageGain);
			break;
		case MW_SETUP_IMPEDANCE:
			eeprom_update_float(&eeprom_impedance_ab, settings.impedance[0]);
			eeprom_update_float(&eeprom_impedance_bc, settings.impedance[1]);
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
	
}

/*
	Установка степени усиления АЦП
*/
void setAdsGainByIndex(Adafruit_ADS1115* _ads, byte _index) {
	Serial.print(F("ADS: "));
	switch(_index) {
		case 1:
			Serial.println(F("GAIN_ONE"));
			_ads->setGain(GAIN_ONE);
			break;
		case 2:
			Serial.println(F("GAIN_TWO"));
			_ads->setGain(GAIN_TWO);
			break;
		case 3:
			Serial.println(F("GAIN_FOUR"));
			_ads->setGain(GAIN_FOUR);
			break;
		case 4:
			Serial.println(F("GAIN_EIGHT"));
			_ads->setGain(GAIN_EIGHT);
			break;
		case 5:
			Serial.println(F("GAIN_SIXTEEN"));
			_ads->setGain(GAIN_SIXTEEN);
			break;
		default:
			Serial.println(F("GAIN_TWOTHIRDS"));
			_ads->setGain(GAIN_TWOTHIRDS);
			break;
	}
}

/*
	Инициализация усиления АЦП по напряжению
*/
void initAdsVoltageGain() {
	setAdsGainByIndex(&adsVoltage, settings.currentVoltageGain);
	ads.voltageStep = ads.gainStep[settings.currentVoltageGain] / 1000.0;
}

/*
	Инициализация усиления АЦП по току
*/
void initAdsAmperageGain() {
	setAdsGainByIndex(&adsAmperage, settings.currentAmperageGain);
	ads.amperageStep = ads.gainStep[settings.currentAmperageGain] / 1000.0;
}