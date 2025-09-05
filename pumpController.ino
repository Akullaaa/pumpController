// Структура для статистики выполнения
struct ProcessStats {
  unsigned long callCount = 0;
  unsigned long totalTime = 0;
  unsigned long minTime = 0xFFFFFFFF; // Максимальное значение unsigned long
  unsigned long maxTime = 0;
  
  void update(unsigned long executionTime) {
    callCount++;
    totalTime += executionTime;
    if (executionTime < minTime) minTime = executionTime;
    if (executionTime > maxTime) maxTime = executionTime;
  }
  
  unsigned long getAverageTime() const {
    return callCount > 0 ? totalTime / callCount : 0;
  }
  
  String getStatsString() const {
    return "(" + String(callCount) + 
           ") " + String(minTime) + 
           "/" + String(maxTime) + 
           "/" + String(getAverageTime());
  }
};

// Функция для получения свободной оперативной памяти
int getFreeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
};

// Глобальная статистика для всех насосов
ProcessStats globalProcessStats;

// Глобальная переменная для учета общего времени, проведенного в сне
volatile unsigned long totalSleepTime = 0;

// Функция для получения текущего времени с учетом времени сна
unsigned long getCurrentTime() {
  return millis() + totalSleepTime;
}

// Функция для перевода Arduino в спящий режим
void enterSleepMode(unsigned long sleepTime) {
  // Отключаем АЦП для экономии энергии
  byte adcsra = ADCSRA;
  ADCSRA = 0;
  
  // Настраиваем время сна через watchdog timer
  noInterrupts();
  
  // Сбрасываем watchdog
  MCUSR &= ~(1 << WDRF);
  
  // Устанавливаем режим watchdog и интервал
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  
  // Устанавливаем интервал сна (компактная версия)
  WDTCSR = (1 << WDIE) | (sleepTime > 4096 ? 0b111 : sleepTime > 2048 ? 0b101 : 
            sleepTime > 1024 ? 0b100 : sleepTime > 512 ? 0b011 : sleepTime > 256 ? 0b010 : 
            sleepTime > 128 ? 0b001 : sleepTime > 64 ? 0b000 : sleepTime > 32 ? 0b11 : 
            sleepTime > 16 ? 0b10 : 0b01);
  
  // Устанавливаем режим сна (POWER_DOWN)
  SMCR = (1 << SM1) | (1 << SM0); // SLEEP_MODE_PWR_DOWN
  SMCR |= (1 << SE); // Включаем сон
  
  // Включаем прерывания и переходим в сон
  interrupts();
  asm volatile("sleep\n\t");
  
  // Проснулись - отключаем сон
  SMCR &= ~(1 << SE);
  
  // Восстанавливаем АЦП
  ADCSRA = adcsra;
  
  // Добавляем время сна к общему счетчику
  totalSleepTime += sleepTime;
}

// Обработчик прерывания watchdog (должен быть объявлен, даже если пустой)
void WDT_vect(void) __attribute__((naked));
void WDT_vect(void) {
  asm volatile("reti\n\t");
}

// Класс для датчика влажности
class SoilSensor {
private:
  int soilPin;              // Пин датчика влажности
  int dryThreshold;         // Порог сухости
  unsigned long checkInterval; // Интервал проверки
  unsigned long lastCheckTime; // Время последней проверки

public:
  // Конструктор
  SoilSensor(int soilPin, int dryThreshold, unsigned long checkInterval)
    : soilPin(soilPin), dryThreshold(dryThreshold), checkInterval(checkInterval), lastCheckTime(0) {}

  // Метод для проверки, пришло ли время для следующей проверки
  bool isCheckTime() {
    return getCurrentTime() - lastCheckTime >= checkInterval;
  }

  // Метод для чтения влажности
  int readMoisture() {
    // Обновляем время последней проверки
    lastCheckTime = getCurrentTime();
    return analogRead(soilPin);
  }

  // Метод для проверки необходимости полива
  bool needsWatering(int moisture) {
    return moisture >= dryThreshold;
  }

  // Метод для получения времени до следующей проверки
  unsigned long getTimeToNextCheck() const {
    unsigned long timeSinceLastCheck = getCurrentTime() - lastCheckTime;
    if (timeSinceLastCheck >= checkInterval) {
      return 0; // Уже пора проверять
    }
    return checkInterval - timeSinceLastCheck;
  }

  // Метод для получения информации о датчике
  String getInfo(int moisture, int getPumpPin) {
    unsigned long timeToNextCheck = getTimeToNextCheck();
    int difference = moisture - dryThreshold;
    String status = difference >= 0 ? "НУЖЕН ПОЛИВ" : "НОРМА";
    
    return String(getCurrentTime() /1000) + " сек. - Датчик " + String(soilPin) + 
           "(p="+ String(getPumpPin)+"): влажность=" + String(moisture) + 
           " | Порог: " + String(dryThreshold) +
           " | Разница: " + String(difference) +
           " | Статус: " + status +
           " | До проверки: " + String(timeToNextCheck / 1000) + " сек." +
           " | " + globalProcessStats.getStatsString() +
           " | RAM: " + String(getFreeMemory()) + " байт";
  }

  // Геттеры
  int getSoilPin() const { return soilPin; }
  int getDryThreshold() const { return dryThreshold; }
  unsigned long getCheckInterval() const { return checkInterval; }
  unsigned long getLastCheckTime() const { return lastCheckTime; }
};

// Класс для управления насосом
class Pump {
private:
  SoilSensor sensor;        // Объект датчика (копия)
  int pumpPin;              // Пин реле насоса
  unsigned long pumpDuration; // Длительность работы насоса
  bool isPumping;           // Флаг активности насоса
  unsigned long pumpStartTime; // Время начала работы насоса

public:
  // Конструктор принимает временный объект SoilSensor
  Pump(SoilSensor sensor, int pumpPin, unsigned long pumpDuration)
    : sensor(sensor), pumpPin(pumpPin), pumpDuration(pumpDuration), 
      isPumping(false), pumpStartTime(0) {}

  // Метод для настройки пина насоса
  void setupPin() {
    pinMode(pumpPin, OUTPUT);
  }

  // Метод для проверки, истекло ли время работы насоса
  bool isPumpTimeExpired() {
    return getCurrentTime() - pumpStartTime >= pumpDuration;
  }

  // Метод для включения насоса
  void startPumping() {
    digitalWrite(pumpPin, LOW);  // Инвертированная логика: LOW включает насос
    isPumping = true;
    pumpStartTime = getCurrentTime();
    Serial.println("Насос на пине " + String(pumpPin) + " включен s=" + String(getSensor().getSoilPin()) + "");
  }

  // Метод для выключения насоса
  void stopPumping() {
    digitalWrite(pumpPin, HIGH); // Инвертированная логика: HIGH выключает насос
    isPumping = false;
    Serial.println("Насос на пине " + String(pumpPin) + " выключен");
  }

  // Основной метод обработки насоса
  void process() {
    unsigned long startTime = micros(); // Засекаем время начала
    
    // Проверяем, не активен ли насос в данный момент
    if (isPumping) {
      // Если насос работает, проверяем, не истекло ли время работы
      if (isPumpTimeExpired()) {
        stopPumping();
      }
      
      unsigned long endTime = micros(); // Засекаем время окончания
      globalProcessStats.update(endTime - startTime); // Обновляем статистику
      return; // Прерываем выполнение, если насос активен
    }
    
    // Проверяем, пришло ли время для следующей проверки датчика
    if (sensor.isCheckTime()) {
      int moisture = sensor.readMoisture();
      
      // Выводим информацию в монитор порта
      Serial.println(sensor.getInfo(moisture, getPumpPin()));

      // Проверяем, слишком ли сухая почва
      if (sensor.needsWatering(moisture)) {
        startPumping();
      }
    }
    
    unsigned long endTime = micros(); // Засекаем время окончания
    globalProcessStats.update(endTime - startTime); // Обновляем статистику
  }

  // Геттеры
  bool getIsPumping() const { return isPumping; }
  unsigned long getPumpStartTime() const { return pumpStartTime; }
  unsigned long getPumpDuration() const { return pumpDuration; }
  SoilSensor getSensor() const { return sensor; }
  int getPumpPin() const { return pumpPin; }
};

// Класс контроллера насосов
class PumpController {
private:
  Pump** pumps;           // Массив указателей на насосы
  int pumpCount;          // Количество насосов
  int maxPumps;           // Максимальное количество насосов
  bool wasSleeping;       // Флаг, что только что вышли из сна

public:
  // Конструктор
  PumpController(int maxPumps = 10) : pumpCount(0), maxPumps(maxPumps), wasSleeping(false) {
    pumps = new Pump*[maxPumps];
    for (int i = 0; i < maxPumps; i++) {
      pumps[i] = nullptr;
    }
  }

  // Деструктор для очистки памяти
  ~PumpController() {
    for (int i = 0; i < pumpCount; i++) {
      delete pumps[i];
    }
    delete[] pumps;
  }

  // Метод для добавления насоса
  bool addPump(SoilSensor sensor, int pumpPin, unsigned long pumpDuration) {
    if (pumpCount >= maxPumps) {
      Serial.println("Ошибка: достигнуто максимальное количество насосов");
      return false;
    }
    
    pumps[pumpCount] = new Pump(sensor, pumpPin, pumpDuration);
    pumpCount++;
    return true;
  }

  // Метод инициализации для раздела setup
  void setup() {
    for (int i = 0; i < pumpCount; i++) {
      pumps[i]->setupPin();
      pumps[i]->stopPumping();
    }
    Serial.println("Система автоматического полива инициализирована");
    Serial.println("Количество насосов: " + String(pumpCount));
    Serial.println("Свободная память: " + String(getFreeMemory()) + " байт");
  }

  // Метод для получения насоса по индексу
  Pump* getPump(int index) {
    if (index >= 0 && index < pumpCount) {
      return pumps[index];
    }
    return nullptr;
  }

  // Метод для вывода информации о всех датчиках
  void printAllSensorsInfo() {
    Serial.println("=== ПРОБУЖДЕНИЕ ===");
    Serial.println("Общее время работы: " + String(getCurrentTime() / 1000) + " сек.");
    Serial.println("Время в сне: " + String(totalSleepTime / 1000) + " сек.");
    Serial.println("Текущее время millis(): " + String(millis() / 1000) + " сек.");
    Serial.println("----------------------------------------");
    
    for (int i = 0; i < pumpCount; i++) {
      Pump* pump = pumps[i];
      SoilSensor sensor = pump->getSensor();
      
      // Читаем текущую влажность
      int moisture = analogRead(sensor.getSoilPin());
      
      // Вычисляем разницу с порогом
      int difference = moisture - sensor.getDryThreshold();
      
      // Получаем время до следующей проверки (используем метод из класса Sensor)
      unsigned long timeToNextCheck = sensor.getTimeToNextCheck();
      
      // Формируем строку статуса
      String status = difference >= 0 ? "НУЖЕН ПОЛИВ" : "НОРМА";
      
      Serial.println("Датчик A" + String(sensor.getSoilPin() - A0) + 
                     " -> Насос D" + String(pump->getPumpPin()) +
                     " | Влажность: " + String(moisture) +
                     " | Порог: " + String(sensor.getDryThreshold()) +
                     " | Разница: " + String(difference) +
                     " | Статус: " + status +
                     " | До проверки: " + String(timeToNextCheck / 1000) + " сек.");
    }
    
    Serial.println("----------------------------------------");
    Serial.println("Статистика: " + globalProcessStats.getStatsString());
    Serial.println("Свободная RAM: " + String(getFreeMemory()) + " байт");
    Serial.println("========================================");
  }

  // Метод для вычисления времени до следующей проверки
  unsigned long calculateSleepTime() {
    unsigned long minTimeToNextCheck = 0xFFFFFFFF;
    unsigned long currentTime = getCurrentTime();
    
    for (int i = 0; i < pumpCount; i++) {
      SoilSensor sensor = pumps[i]->getSensor();
      unsigned long timeToNextCheck = sensor.getTimeToNextCheck();
      
      if (timeToNextCheck < minTimeToNextCheck) {
        minTimeToNextCheck = timeToNextCheck;
      }
    }
    
    // Ограничиваем время сна 8 секундами (максимум для watchdog)
    if (minTimeToNextCheck > 8000) {
      return 8000;
    }
    return minTimeToNextCheck;
  }

  // Метод process для loop
  void process() {
    // Если только что вышли из сна, выводим информацию о датчиках
    if (wasSleeping) {
      printAllSensorsInfo();
      wasSleeping = false;
    }
    
    // Обрабатываем все насосы
    for (int i = 0; i < pumpCount; i++) {
      pumps[i]->process();
    }
    
    // Вычисляем время до следующей проверки и переходим в сон
    unsigned long sleepTime = calculateSleepTime();
    if (sleepTime > 0) {
      // Не спим, если какой-то насос активен
      for (int i = 0; i < pumpCount; i++) {
        if (pumps[i]->getIsPumping()) {
          return;
        }
      }
      
      // Переходим в спящий режим
      enterSleepMode(sleepTime);
      // Устанавливаем флаг, что только что вышли из сна
      wasSleeping = true;
    }
  }

  // Геттеры
  int getPumpCount() const { return pumpCount; }
  int getMaxPumps() const { return maxPumps; }
};

// Создание контроллера насосов
PumpController pumpController(10); // Максимум 10 насосов

void setup() {
  // Инициализация последовательного порта
  Serial.begin(9600);
  
  // Добавление насосов в контроллер
  pumpController.addPump(SoilSensor(A0, 800, 7000), 2, 2000);  // Насос 1
  pumpController.addPump(SoilSensor(A1, 275, 7000), 3, 2000);   // Насос 2
  pumpController.addPump(SoilSensor(A2, 600, 700000), 4, 2000);  // Насос 3
  pumpController.addPump(SoilSensor(A3, 700, 7000000), 5, 2000);  // Насос 4
  
  // Инициализация контроллера
  pumpController.setup();
}

void loop() {
  // Обработка всех насосов через контроллер
  pumpController.process();
}