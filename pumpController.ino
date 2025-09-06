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
}

// Глобальная статистика для всех насосов
ProcessStats globalProcessStats;

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
    return millis() - lastCheckTime >= checkInterval;
  }

  // Метод для чтения влажности
  int readMoisture() {
    // Обновляем время последней проверки
    lastCheckTime = millis();
    return analogRead(soilPin);
  }

  // Метод для проверки необходимости полива
  bool needsWatering(int moisture) {
    return moisture >= dryThreshold;
  }

  // Метод для получения информации о датчике
  String getInfo(int moisture, int getPumpPin) {
    return String(millis() /1000) + " сек. - Датчик " + String(soilPin) + 
           "(p="+ String(getPumpPin)+"): влажность=" + String(moisture) + 
           "(" + String(dryThreshold) +
           ")" + String(moisture - dryThreshold) +
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
    return millis() - pumpStartTime >= pumpDuration;
  }

  // Метод для включения насоса
  void startPumping() {
    digitalWrite(pumpPin, LOW);  // Инвертированная логика: LOW включает насос
    isPumping = true;
    pumpStartTime = millis();
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

public:
  // Конструктор
  PumpController(int maxPumps = 10) : pumpCount(0), maxPumps(maxPumps) {
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

  // Метод process для loop
  void process() {
    for (int i = 0; i < pumpCount; i++) {
      pumps[i]->process();
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
  pumpController.addPump(SoilSensor(A0, 800, 420000), 2, 2000);  // Насос 1
  pumpController.addPump(SoilSensor(A1, 277, 16000), 3, 2000);   // Насос 2
  pumpController.addPump(SoilSensor(A2, 600, 700000), 4, 2000);  // Насос 3
  pumpController.addPump(SoilSensor(A3, 700, 7000000), 5, 2000);  // Насос 4
  
  // Инициализация контроллера
  pumpController.setup();
}

void loop() {
  // Обработка всех насосов через контроллер
  pumpController.process();
}
