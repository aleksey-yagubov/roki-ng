/*
  Проект "androSot"
  Автор: Зубр ОВК
  Описание: движок ходьбы по методу Азера с дистанционным управлением.

  ============================================================
  ВЕРСИЯ ИСПРАВЛЕНИЙ: v3.0 (04.11.2026)
  ============================================================
  v3.0 - kick(): наклон корпуса при ударе 0.04 вперед -> -0.03 НАЗАД
         (инерция замаха и так тянет вперед - заваливался), mixing
         отключается на время удара (как в прыжках); jump_left():
         боковой толчок 200 -> 240 * factor (часть энергии уходит на
         компенсацию наклона - прыгал короче, чем вправо).
  v2.9 - jump_left(): стопа возвращается в плоское положение еще в
         фазе полета (раньше носок оставался вывернут при приземлении -
         робот не возвращался в исходную и застревал в наклоне назад);
         перед включением mixing добавлена пауза 20 фреймов на
         стабилизацию после приземления.
  v2.8 - jump_left(): при прыжке влево робот заваливался назад и
         падал (код зеркален правому - причина физическая). В фазу
         приседания добавлен наклон вперед: FOOT_FRONT = 8*factor.
         Если начнет кидать ВПЕРЕД - поменять знак на минус.
  v2.7 - серия с паузой при зажатой кнопке перенесена с прыжков на
         ПОВОРОТЫ (A/Y/X/B): пока кнопка зажата - поворот, пауза
         REPEAT_PAUSE=25 фреймов, снова поворот. Прыжки возвращены
         к одиночному срабатыванию.
  v2.6 - боковой шаг разделен: вправо 18 мм (ходит хорошо), влево
         24 мм (STEP_SIDE_LEN_L) - робот физически несимметричен,
         влево 18 мм топтался на месте. Прыжки: при зажатой кнопке -
         серия прыжков с паузой JUMP_PAUSE=25 фреймов между ними.
  v2.5 - боковой шаг 12 -> 18 мм (12 минус боковая коррекция 3.3 мм
         давали ~9 мм на шаг - "на месте"). Прыжки: на время прыжка
         mixing() отключается (leftRightFactor/frontBackFactor = 0,
         после - обратно 100/70) - гиростабилизация дергала стопы
         при приседании и сбивала толчок. В баскетболе прыжков нет,
         поэтому там mixing никому не мешал.
  v2.4 - пульт: выбор ГЛАВНОГО направления стика. Робот "топтался на
         месте", потому что при стике чуть по диагонали каждый шаг
         добавлялся фиксированный боковой шаг +-12 мм - ноги мяли пол
         вбок вместо шага вперед. Теперь: что отклонено сильнее, то и
         выполняется (вперед/назад ИЛИ вбок), второе направление = 0.
         Прямая ходьба всегда чистая, как в баскетболе.
  v2.3 - НАЙДЕНА настоящая причина "баскетбол ходит лучше": в его
         action() ПЕРЕД ходьбой параметры переопределяются:
         gaitHeight=165 (не 180), stepHeight=20 (не 32),
         bodyTiltAtWalk=0 (не 0.04), шаг 25 мм. Значения 180/32/0.04
         из setup() там не используются. Перенесены реальные значения:
         setup(), walk_params_to_normal() -> 165/20/0; тест STEP_LENGHT
         50 -> 25 (был шаг вдвое длиннее баскетбольного).
  v2.2 - ПРОСТАЯ пультовая ходьба, как в баскетболе: длина шага
         больше НЕ зависит от отклонения стика. Стик задает только
         направление, шаг фиксированный: вперед 25 мм, назад 20 мм,
         вбок 12 мм, поворот 0.15 рад (константы STEP_FORWARD_LEN,
         STEP_BACK_LEN, STEP_SIDE_LEN, STEP_TURN - меняются в одном
         месте, перед joystickToStepLength). Раньше дрожание стика
         делало шаги разной длины - робота болтало и дёргало.
  v2.1 - походка приведена к рабочему скрипту "Баскетбол Roki-1"
         ПОЛНОСТЬЮ, по требованию: mixing() (гиростабилизация крена)
         ВКЛЮЧЕНА (sfCreateTask в main), testDrop() восстановлен
         оригинальный (пауза 1 сек при падении), переходы в/из позы
         ходьбы снова 40 фреймов (walkInitialPoseFine/walkFinalPoseFine).
         Фазы walkPhasa1-4, walkInit, walkCycle, stabilizeRotationByIMU,
         mixing - сверены с баскетбольным скриптом, идентичны.
         Оставлены только пультовые исправления, которых в баскетболе
         нет и быть не может: мертвая зона X джойстика, гистерезис,
         перезахват курса при старте ходьбы с пульта.
  v2.0 - параметры походки перенесены из рабочего скрипта
         "Баскетбол Roki-1" (тот же движок Азера, проверен на
         этом роботе): stepHeight 42 -> 32, bodyTiltAtWalk 0 -> 0.04,
         selfMotionShiftCorrectionY 0 -> 70/21 (~3.3 мм в шаг),
         коэффициент курса в walkInit() 0.5 -> 0.25 (оригинал),
         walkFinalPoseFine(): явный наклон xr=xl=bodyTiltAtWalk,
         убрана моя добавка rotation += 0.06 (в рабочем коде её нет),
         стартовый курс main(): get_yaw() - 0.02 (как в рабочем).
         ВСЕ логические исправления v1.x сохранены: мертвая зона X
         джойстика, гистерезис, testDrop выкл., перезахват курса при
         старте ходьбы, walkLoopFast, тест runTest(). mixing() по-прежнему
         выключена (у этого экземпляра с ней ходьба дёрганая).
         Если снова потянет влево - вернуть rotation += 0.02..0.06
         в stabilizeRotationByIMU().
  v1.0 - walkLoopFast(): быстрая ходьба управляется джойстиком
         (было захардкожено stepLength=0 - ходила только боком),
         исправлены пропущенные {} - return срабатывал всегда;
         kick(): добавлены пропущенные {}.
  v1.1 - selfMotionShiftCorrectionY = 0: увод влево оказался не боковым
         смещением, а подворотом корпуса - боковая компенсация убрана;
         stabilizeRotationByIMU(): постоянная компенсация подворота
         влево (rotation += 0.06); walkInit(): усиление курса 0.25 -> 0.5;
         bodyTiltAtWalk = 0 (было 0.01 - заваливался вперед);
         main(): переключатель testForward (0 - пульт, 1 - тест вперед).
  v1.2 - чистка кода: удален мертвый код и неиспользуемые переменные,
         проверка "джойстик отпущен" вынесена в joystickReleased().
         Логика не менялась.
  v1.3 - stepHeight 42 -> 48 (нога не шаркала по полу);
         walking_frame 2 -> 3 (команды серв успевают завершиться,
         меньше раскачка); то же в walk_params_to_normal() и WALK_SLOW;
         joystickToSideLength(): мертвая зона по X - первая нога больше
         не переключается от шума стика (из-за этого ломалось
         чередование ног, правая нога переставала шагать, уводило влево).
  v1.4 - НЕУДАЧНАЯ, откачена: mixing() (гиростабилизация крена) давала
         дёрганую ходьбу с задержкой; усиление курса 0.65 - перерегулирование.
  v1.5 - mixing() снова выключена, усиление курса возвращено на 0.5;
         amplitude 32 -> 26 против боковой раскачки (меньше перенос веса).
  v1.6 - testDrop() выключена: датчик падения без кода вставания просто
         замораживал робота на 1 сек от вибраций - ложные замирания;
         movingRemoteControl(): курс "вперед" перезапоминается при каждом
         старте ходьбы - робот больше не возвращается к направлению,
         запомненному при включении.
  v1.7 - гистерезис джойстика: вход в ходьбу при отклонении >= 10,
         выход только при < 5 (joystickReleasedWalk/joystickMovedWalk).
         Раньше стик дрожал около 10 -> робот выходил в исходную позу
         (0.4 сек стоит) и входил обратно - "подвисания" на полсекунды.
  v1.8 - переходы в/из позы ходьбы ускорены вдвое: 40 -> 20 фреймов
         (walkInitialPoseFine/walkFinalPoseFine) - меньше задержка
         при каждом нажатии/отпускании джойстика.
  v1.9 - ОТКАТ параметров походки к проверенным значениям v1.1:
         stepHeight 48 -> 42, walking_frame 3 -> 2, amplitude 26 -> 32.
         Изменения v1.3/v1.5 давали судороги ноги (IK на границе
         досягаемости). Логические исправления (мертвая зона X,
         гистерезис, testDrop выкл., перезахват курса) сохранены.
  ============================================================
  Настройки под этот экземпляр робота:
  - параметры походки = РЕАЛЬНЫЕ рабочие значения баскетбола
    (переопределение в его action()): gaitHeight=165, stepHeight=20,
    bodyTiltAtWalk=0, amplitude=32, walking_frame=2, шаг 25 мм,
    selfMotionShiftCorrectionY=70/21, курс 0.25, mixing ВКЛЮЧЕН
  - если потянет влево: rotation += 0.02..0.06 в stabilizeRotationByIMU()
*/
#include <roki1p.h>

#define HARD_BUTTON_R1  32
#define HARD_BUTTON_L1  16

#define MASK_PELVIC     (MASK_LEFT_PELVIC | MASK_RIGHT_PELVIC)
#define MASK_HIP_SIDE   (MASK_LEFT_HIP_SIDE | MASK_RIGHT_HIP_SIDE)
#define MASK_HIP        (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE       (MASK_LEFT_KNEE | MASK_RIGHT_KNEE)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)
#define MASK_FOOT_SIDE  (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE)

#define ARB_GETUP   1048576
#define ARB_SPLITS  2097152
#define ARB_JUMP    4194304

#define STEP_FIRST  0
#define STEP_LAST   1
#define STEP_OTHER  2

//Параметры шага (задаются джойстиками или тестом)
float stepLength;  // -50 - +70, вперед > 0, назад < 0
float sideLength;  // -20 - +20, вправо > 0, влево < 0
float rotation;    // подворот за шаг, рад: + по часовой, - против

int   slowWalk;    // 0 - обычная походка, 1 - параметры медленной
float selfMotionShiftCorrectionX; // коррекция шага вперед/назад
float selfMotionShiftCorrectionY; // коррекция бокового смещения
int   selfFirstLegIsRightLeg;     // какая нога шагает первой
int   motion_to_right;
float side_motion;
float rotationYieldRight;
float rotationYieldLeft;
int   framestep;

//Положение и ориентация правой стопы
float xtr, ytr, ztr;
float xr, yr, zr, wr;
//Положение и ориентация левой стопы
float xtl, ytl, ztl;
float xl, yl, zl, wl;

float d10;        // расстояние по Y от центра стопы до оси робота

float amplitude;  // боковая амплитуда переноса центра масс
float alpha01;
int   fr1;        // фреймы фазы двойной опоры
int   fr2;        // фреймы фазы переноса ноги
float gaitHeight; // высота центра масс над полом при ходьбе
float stepHeight; // высота подъема стопы
float correctedStepLenght;
float correctedSideLenght;
float correctedSideLenghtHalf;
float ztr0;       // высота стоп в исходной стойке

int   stepType;
int   fps;          // расчетная длительность шага, фреймы
int   walking_frame;
float bodyTiltAtWalk;
float solyLandingSkew;

int   stepNumber;
float yaw, pitch, roll;

float xtl0, xtr0;
float dx0Typical, dy0Typical;
float correctedRotation;
float s;
float fase_offset;
float dx, dy, dx0;
float stepRotation;
float forwardDirection;
int   frameCount;
int   j;

//Факторы миксования по показаниям IMU (для mixing())
int leftRightFactor;
int frontBackFactor;

#define CYCLE_COUNT 20 // тест: количество одинаковых шагов (+3 служебных)
#define STEP_LENGHT 25 // тест: длина шага, мм (как в баскетболе)

//Преобразование кватерниона IMU в углы Эйлера
void quaternion_to_euler_angle() {
  float w = svImuQuaterW / 16384.0;
  float x = svImuQuaterX / 16384.0;
  float y = svImuQuaterY / 16384.0;
  float z = svImuQuaterZ / 16384.0;
  float t0 = +2.0 * (w * x + y * z);
  float t1 = +1.0 - 2.0 * (x * x + y * y);
  float t2 = +2.0 * (w * y - z * x);
  t2 = (t2 > +1.0 ? 1.0 : t2);
  t2 = (t2 < -1.0 ? -1.0 : t2);
  float t3 = +2.0 * (w * z + x * y);
  float t4 = +1.0 - 2.0 * (y * y + z * z);
  pitch = sfFMathATan2(t0, t1);
  roll  = sfFMathASin(t2);
  yaw   = sfFMathATan2(t3, t4);
}

float get_yaw() {
  quaternion_to_euler_angle();
  return yaw;
}

void setup() {
  rotationYieldRight = 0.23;
  rotationYieldLeft  = 0.23;

  //Модель ноги для обратной кинематики
  svIkA5  = 25.2; // от оси симметрии до оси сервы 5
  svIkB5  = 21.8; // от оси сервы 5 до оси сервы 6 по горизонтали
  svIkC5  = 0;    // от оси сервы 6 до нуля Z по вертикали
  svIkA6  = 40;   // от оси сервы 6 до оси сервы 7
  svIkA7  = 64;   // от оси сервы 7 до оси сервы 8
  svIkA8  = 64;   // от оси сервы 8 до оси сервы 9
  svIkA9  = 40;   // от оси сервы 9 до оси сервы 10
  svIkA10 = 12.5; // от оси сервы 10 до центра стопы по горизонтали
  svIkB10 = 24;   // от оси сервы 10 до низа стопы
  svIkC10 = 0;    // от оси сервы 6 до оси сервы 10 по горизонтали

  d10 = 59.5;     // расстояние по Y от центра стопы до оси робота

  //Ограничения серв в радианах относительно нуля
  svIkLimA5min  = -150 * MATH_RAD_PER_DEGREE;
  svIkLimA5max  =  150 * MATH_RAD_PER_DEGREE;
  svIkLimA6min  =  -90 * MATH_RAD_PER_DEGREE;
  svIkLimA6max  =   17 * MATH_RAD_PER_DEGREE;
  svIkLimA7min  =  -97 * MATH_RAD_PER_DEGREE;
  svIkLimA7max  =  103 * MATH_RAD_PER_DEGREE;
  svIkLimA8min  = -158 * MATH_RAD_PER_DEGREE;
  svIkLimA8max  =   32 * MATH_RAD_PER_DEGREE;
  svIkLimA9min  = -122 * MATH_RAD_PER_DEGREE;
  svIkLimA9max  =   83 * MATH_RAD_PER_DEGREE;
  svIkLimA10min =  -83 * MATH_RAD_PER_DEGREE;
  svIkLimA10max =   20 * MATH_RAD_PER_DEGREE;
  sfIkSetup();

  framestep  = 2;
  gaitHeight = 165;   // как в action() баскетбола - реальные рабочие значения,
  stepHeight = 20.0;  // а не 180/32 из setup() (они там сразу переопределяются)

  selfMotionShiftCorrectionX = -50.0 / 21.0;
  selfMotionShiftCorrectionY =  70.0 / 21.0; // как в рабочем скрипте (~3.3 мм в шаг)

  ztr0 = -(svIkC5 + svIkA6 + svIkA7 + svIkA8 + svIkA9 + svIkB10 - 1);
  zr = zl = -1;

  stepLength = 0.0;
  sideLength = 0.0;
  rotation   = 0;
  selfFirstLegIsRightLeg = 1;
  motion_to_right = 1;
  side_motion = 0.0;

  bodyTiltAtWalk  = 0.0; // в баскетболе перед ходьбой ставится 0 (action())
  solyLandingSkew = 0.01;

  if( slowWalk ) {
    fr1 = 50;       // фреймы фазы двух ног на полу
    fr2 = 20;       // фреймы фазы одной ноги в воздухе
    amplitude = 110;
  }
  else {
    fr1 = 8;
    fr2 = 12;
    amplitude = 32; // боковая амплитуда переноса центра масс
  }

  alpha01 = MATH_PI;
  if( fr1 != 0 )
    alpha01 = alpha01 * 2 / fr1;

  //Расчетная длительность шага, фактическая на 1 фрейм короче.
  //walking_frame = 2 - проверенное значение (вариант 3 из v1.3 участвовал
  //в ухудшении ходьбы - откачан)
  fps = 8;
  walking_frame = 2;
}

//Обратная кинематика обеих ног и отправка углов в сервы.
//Линейная интерполяция за fps фреймов, ожидание walking_frame.
int computeAlphaForWalk() {
  //Правая нога
  sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
  if( svIkOutPresent ) {
    if( selfFirstLegIsRightLeg == 1 ) {
      sfPoseGroupLin( MASK_RIGHT_PELVIC,     -svIkEncA5,  fps );
      sfPoseGroupLin( MASK_RIGHT_HIP_SIDE,   -svIkEncA6,  fps );
      sfPoseGroupLin( MASK_RIGHT_HIP,        -svIkEncA7,  fps );
      sfPoseGroupLin( MASK_RIGHT_KNEE,       -svIkEncA8,  fps );
      sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT,  svIkEncA9,  fps );
      sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE,  -svIkEncA10, fps );
    }
    else {
      sfPoseGroupLin( MASK_LEFT_PELVIC,       svIkEncA5,  fps );
      sfPoseGroupLin( MASK_LEFT_HIP_SIDE,    -svIkEncA6,  fps );
      sfPoseGroupLin( MASK_LEFT_HIP,         -svIkEncA7,  fps );
      sfPoseGroupLin( MASK_LEFT_KNEE,        -svIkEncA8,  fps );
      sfPoseGroupLin( MASK_LEFT_FOOT_FRONT,   svIkEncA9,  fps );
      sfPoseGroupLin( MASK_LEFT_FOOT_SIDE,   -svIkEncA10, fps );
    }
  }
  //Левая нога
  sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
  if( svIkOutPresent ) {
    if( selfFirstLegIsRightLeg == 1 ) {
      sfPoseGroupLin( MASK_LEFT_PELVIC,      -svIkEncA5,  fps );
      sfPoseGroupLin( MASK_LEFT_HIP_SIDE,    -svIkEncA6,  fps );
      sfPoseGroupLin( MASK_LEFT_HIP,         -svIkEncA7,  fps );
      sfPoseGroupLin( MASK_LEFT_KNEE,        -svIkEncA8,  fps );
      sfPoseGroupLin( MASK_LEFT_FOOT_FRONT,   svIkEncA9,  fps );
      sfPoseGroupLin( MASK_LEFT_FOOT_SIDE,   -svIkEncA10, fps );
    }
    else {
      sfPoseGroupLin( MASK_RIGHT_PELVIC,      svIkEncA5,  fps );
      sfPoseGroupLin( MASK_RIGHT_HIP_SIDE,   -svIkEncA6,  fps );
      sfPoseGroupLin( MASK_RIGHT_HIP,        -svIkEncA7,  fps );
      sfPoseGroupLin( MASK_RIGHT_KNEE,       -svIkEncA8,  fps );
      sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT,  svIkEncA9,  fps );
      sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE,  -svIkEncA10, fps );
    }
  }
  sfWaitFrame( walking_frame );
  return 1;
}

//То же, но плавное достижение конечной позы за frames фреймов
void computeAlphaForWalkFine( int frames ) {
  sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
  if( svIkOutPresent ) {
    if( selfFirstLegIsRightLeg == 1 ) {
      sfPoseGroupLin( MASK_RIGHT_PELVIC,     -svIkEncA5,  frames );
      sfPoseGroupLin( MASK_RIGHT_HIP_SIDE,   -svIkEncA6,  frames );
      sfPoseGroupLin( MASK_RIGHT_HIP,        -svIkEncA7,  frames );
      sfPoseGroupLin( MASK_RIGHT_KNEE,       -svIkEncA8,  frames );
      sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT,  svIkEncA9,  frames );
      sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE,  -svIkEncA10, frames );
    }
    else {
      sfPoseGroupLin( MASK_LEFT_PELVIC,       svIkEncA5,  frames );
      sfPoseGroupLin( MASK_LEFT_HIP_SIDE,    -svIkEncA6,  frames );
      sfPoseGroupLin( MASK_LEFT_HIP,         -svIkEncA7,  frames );
      sfPoseGroupLin( MASK_LEFT_KNEE,        -svIkEncA8,  frames );
      sfPoseGroupLin( MASK_LEFT_FOOT_FRONT,   svIkEncA9,  frames );
      sfPoseGroupLin( MASK_LEFT_FOOT_SIDE,   -svIkEncA10, frames );
    }
  }
  sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
  if( svIkOutPresent ) {
    if( selfFirstLegIsRightLeg == 1 ) {
      sfPoseGroupLin( MASK_LEFT_PELVIC,      -svIkEncA5,  frames );
      sfPoseGroupLin( MASK_LEFT_HIP_SIDE,    -svIkEncA6,  frames );
      sfPoseGroupLin( MASK_LEFT_HIP,         -svIkEncA7,  frames );
      sfPoseGroupLin( MASK_LEFT_KNEE,        -svIkEncA8,  frames );
      sfPoseGroupLin( MASK_LEFT_FOOT_FRONT,   svIkEncA9,  frames );
      sfPoseGroupLin( MASK_LEFT_FOOT_SIDE,   -svIkEncA10, frames );
    }
    else {
      sfPoseGroupLin( MASK_RIGHT_PELVIC,      svIkEncA5,  frames );
      sfPoseGroupLin( MASK_RIGHT_HIP_SIDE,   -svIkEncA6,  frames );
      sfPoseGroupLin( MASK_RIGHT_HIP,        -svIkEncA7,  frames );
      sfPoseGroupLin( MASK_RIGHT_KNEE,       -svIkEncA8,  frames );
      sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT,  svIkEncA9,  frames );
      sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE,  -svIkEncA10, frames );
    }
  }
  sfWaitFrame( frames );
}

//Проверка падения по акселерометру (оригинал из рабочего скрипта Баскетбол)
void testDrop() {
  if( sfAbs(svImuAccX) > 50000 || sfAbs(svImuAccZ) > 50000 ) {
    //Произошло падение - пауза 1 сек
    sfWaitFrame(100);
  }
}

//Начальная поза ходьбы (плавно, из любого состояния)
void walkInitialPoseFine() {
  xtr = xtl = 0;
  ztr = ztl = -gaitHeight;
  ytr = -d10 - amplitude / 2.0;
  ytl =  d10 - amplitude / 2.0;
  computeAlphaForWalkFine( 40 ); // 40 фреймов, как в рабочем скрипте Баскетбол
}

//Исходная стойка (ровно, на высоте ztr0), плавно
void walkFinalPoseFine() {
  xtr = xtl = 0;
  ztr = ztl = -(svIkC5 + svIkA6 + svIkA7 + svIkA8 + svIkA9 + svIkB10 - 1);
  ytr = -d10;
  ytl =  d10;
  xr = xl = bodyTiltAtWalk; // как в рабочем скрипте - не сбрасывать наклон в ноль
  computeAlphaForWalkFine( 40 ); // 40 фреймов, как в рабочем скрипте Баскетбол
}

//Подготовка следующего цикла ходьбы
void walkInit() {
  //Коррекция длины шага и бокового движения
  correctedStepLenght      = stepLength + selfMotionShiftCorrectionX;
  correctedSideLenght      = sideLength - selfMotionShiftCorrectionY;
  correctedSideLenghtHalf  = correctedSideLenght / 2;

  //Коррекция поворота. Коэффициент 0.25 - оригинал из рабочего скрипта
  //(усиление до 0.5/0.65 в v1.x давало перерегулирование курса)
  correctedRotation = -rotation * 0.25 * 0.23 / ( rotation <= 0 ? rotationYieldRight : rotationYieldLeft );

  xtl0 = correctedStepLenght * (1   - (fr1 + fr2 + 2.0 * framestep) / (2 * fr1 + fr2 + 2.0 * framestep)) * 1.5;
  xtr0 = correctedStepLenght * (0.5 - (fr1 + fr2 + 2.0 * framestep) / (2 * fr1 + fr2 + 2.0 * framestep));
  dx0Typical = correctedStepLenght / (2 * fr1 + fr2 + 2.0 * framestep) * framestep;
  dy0Typical = correctedSideLenght / (2.0 * fr2) * framestep;

  xr = xl = bodyTiltAtWalk;
  yl =  solyLandingSkew;
  yr = -solyLandingSkew;
  fase_offset = 0.7;
}

//Фаза 1: двойная опора, перенос центра масс
void walkPhasa1() {
  ztl = ztr = -gaitHeight;
  for( j = 0; j < fr1; j += framestep ) {
    s = (amplitude / 2 + correctedSideLenghtHalf) * sfMathCos(alpha01 * (j / 2 + fase_offset * framestep));
    ytr = s - d10 + correctedSideLenghtHalf;
    ytl = s + d10 + correctedSideLenghtHalf;
    if( stepType == STEP_FIRST ) continue;
    xtl = xtl0 - dx0Typical - dx0Typical * j / framestep;
    xtr = xtr0 - dx0Typical - dx0Typical * j / framestep;
    if( !computeAlphaForWalk() )
      return;
    testDrop();
  }
}

//Фаза 3: двойная опора, вторая половина цикла
void walkPhasa3() {
  ztl = ztr = -gaitHeight;
  for( j = 0; j < fr1; j += framestep ) {
    s = (amplitude / 2 + correctedSideLenghtHalf) * sfMathCos(alpha01 * ((j - fr2) / 2 + fase_offset * framestep));
    ytr = s - d10 - correctedSideLenghtHalf;
    ytl = s + d10 + correctedSideLenghtHalf;
    xtl -= dx0Typical;
    xtr -= dx0Typical;
    if( !computeAlphaForWalk() )
      return;
    testDrop();
  }
}

//Фаза 2: перенос правой ноги
void walkPhasa2() {
  ztr = -gaitHeight + stepHeight;
  dy = correctedSideLenght * framestep / (fr2 - 2.0 * framestep);
  stepRotation = correctedRotation / (fr2 - 2.0 * framestep) * 2;

  if( stepType == STEP_FIRST )
    dx = correctedStepLenght / (fr2 - 2.0 * framestep) * framestep / 2;
  else
    dx = correctedStepLenght / (fr2 - 2.0 * framestep) * framestep;

  for( j = 0; j < fr2; j += framestep ) {
    if( j == 0 ) {
      xtr -= dx0Typical;
      ytr  = s - d10 + dy0Typical;
    }
    else if( j == (fr2 - framestep) ) {
      xtr -= dx0Typical;
      ytr  = s - d10 + 2 * dy0Typical - correctedSideLenght;
    }
    else {
      xtr += dx;
      ytr  = s - 64 + dy0Typical - dy * (j / 2);
      wr = correctedRotation - j * stepRotation;
      wl = wr;
    }
    xtl -= dx0Typical;
    ytl += dy0Typical;
    if( !computeAlphaForWalk() )
      return;
    testDrop();
  }
}

//Фаза 4: перенос левой ноги
void walkPhasa4() {
  stepRotation = correctedRotation / (fr2 - 2.0 * framestep) * 2;
  ztl = -gaitHeight + stepHeight;
  if( stepType == STEP_LAST ) {
    dx0 = dx0Typical * 4 / fr2;
    dx = (correctedStepLenght * (fr1 + fr2) / (4 * fr1) + 2.0 * dx0) / (fr2 - 2.0 * framestep) * framestep / 1.23076941;
  }
  else {
    dx  = correctedStepLenght / (fr2 - 2.0 * framestep) * framestep;
    dx0 = dx0Typical;
  }
  for( j = 0; j < fr2; j += framestep ) {
    if( stepType == STEP_LAST && j == (fr2 - framestep) ) {
      ztl = -gaitHeight;
      ytl = s + d10;
    }
    if( j == 0 ) {
      xtl -= dx0;
      ytl  = s + 64 + correctedSideLenghtHalf;
    }
    else if( j == (fr2 - framestep) ) {
      xtl -= dx0;
      ytl  = s + d10 + correctedSideLenghtHalf;
    }
    else {
      xtl += dx;
      ytl  = s + 64 + correctedSideLenghtHalf;
      wr = j * stepRotation - correctedRotation;
      wl = wr;
    }
    xtr -= dx0;
    ytr += dy0Typical;
    if( !computeAlphaForWalk() )
      return;
    testDrop();
  }
}

//Цикл ходьбы: half == 1 - половина цикла, half == 0 - полный цикл
void walkCycle(int half) {
  float xrOld = xr, yrOld = yr;
  float xlOld = xl, ylOld = yl;

  walkInit();
  walkPhasa1();
  walkPhasa2();
  if( half == 0 ) {
    walkPhasa3();
    walkPhasa4();
  }

  xr = xrOld;
  yr = yrOld;
  xl = xlOld;
  yl = ylOld;
}

//Стабилизация направления ходьбы по IMU (как в рабочем скрипте, без добавок)
void stabilizeRotationByIMU() {
  sfQuaternionToEulerImu();
  rotation = forwardDirection - get_yaw();
  if( rotation >  MATH_PI ) rotation -= 2 * MATH_PI;
  if( rotation < -MATH_PI ) rotation += 2 * MATH_PI;
  //Если робота снова потянет влево - раскомментировать и подобрать 0.02..0.06:
  //rotation += 0.06;
  if( rotation >  0.7 ) rotation =  0.7;
  if( rotation < -0.7 ) rotation = -0.7;
}

//Подмешивание показаний гироскопа в стопы (боковой и продольный крен).
//Запускается отдельной задачей: sfCreateTask( mixing, 20 ) в main()
void mixing() {
  leftRightFactor = 100;
  frontBackFactor = 70;
  while(1) {
    leftFootSideAddonMix  = -svImuGyroZ * leftRightFactor >> 10;
    rightFootSideAddonMix =  svImuGyroZ * leftRightFactor >> 10;
    rightFootFrontAddonMix = leftFootFrontAddonMix = -svImuGyroX * frontBackFactor >> 10;
    sfWaitNextFrame();
  }
}

//Тест ходьбы: вперед CYCLE_COUNT шагов по STEP_LENGHT мм, без пульта
void runTest() {
  int i;
  walkInitialPoseFine();

  stepType   = STEP_FIRST;
  stepLength = STEP_LENGHT / 3.0;
  walkCycle(0);

  stepType   = STEP_OTHER;
  stepLength = STEP_LENGHT * 2.0 / 3.0;
  walkCycle(0);

  stepLength = STEP_LENGHT;
  for( i = 0; i < CYCLE_COUNT; i++ ) {
    stabilizeRotationByIMU();
    walkCycle(0);
  }

  stepType = STEP_LAST;
  walkCycle(0);
  walkFinalPoseFine();
}

//Фиксированные шаги (как в баскетболе: 25 мм вперед).
//Стик задает только НАПРАВЛЕНИЕ, длина шага от отклонения стика не зависит -
//робот идет ровно, без рывков от дрожания стика.
#define STEP_FORWARD_LEN  25   // шаг вперед, мм
#define STEP_BACK_LEN     20   // шаг назад, мм
#define STEP_SIDE_LEN     18   // шаг вбок ВПРАВО, мм
#define STEP_SIDE_LEN_L   24   // шаг вбок ВЛЕВО, мм (робот несимметричен - влево нужен шире)
#define STEP_TURN         150  // поворот за шаг (0.150 рад)
#define REPEAT_PAUSE      25   // пауза между поворотами при зажатой кнопке, фреймы

//Джойстик -> длина шага: фиксированная, только направление.
//Работает, только если Y отклонен сильнее X (иначе команда "вбок")
int joystickToStepLength() {
  if( sfAbs(svRemoteRightJoystickY) < 10 ) return 0;
  if( sfAbs(svRemoteRightJoystickY) <= sfAbs(svRemoteRightJoystickX) ) return 0;
  if( -svRemoteRightJoystickY > 0 ) return STEP_FORWARD_LEN;
  return -STEP_BACK_LEN;
}

//Джойстик -> боковой шаг: фиксированный, только направление.
//Работает, только если X отклонен сильнее Y (иначе команда "вперед/назад")
int joystickToSideLength() {
  if( sfAbs(svRemoteRightJoystickX) < 10
   || sfAbs(svRemoteRightJoystickX) <= sfAbs(svRemoteRightJoystickY) ) {
    side_motion = 0;
    return 0; // идем прямо - бокового шага нет, первую ногу не переключаем
  }
  if( svRemoteRightJoystickX > 10 ) {
    side_motion = -STEP_SIDE_LEN;  // вправо
    motion_to_right = 1;
    return STEP_SIDE_LEN;
  }
  side_motion = STEP_SIDE_LEN_L;   // влево (шире - иначе топчется на месте)
  motion_to_right = 0;
  return STEP_SIDE_LEN_L;
}

//Джойстик -> поворот: фиксированный, только направление
int joystickToRotation() {
  if( svRemoteLeftJoystickX > 10 ) return -STEP_TURN;
  if( svRemoteLeftJoystickX < -10 ) return STEP_TURN;
  return 0;
}

//1 - все джойстики отпущены (и не задана серия шагов)
int joystickReleased() {
  return sfAbs( svRemoteRightJoystickX ) < 10
      && sfAbs( svRemoteRightJoystickY ) < 10
      && sfAbs( svRemoteLeftJoystickX  ) < 10
      && !(svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1));
}

//Выход из ходьбы - с гистерезисом: порог ниже, чем вход (10).
//Иначе при медленной ходьбе стик дрожал около 10, робот каждый раз
//выходил в исходную позу (0.4 сек стоит) и входил обратно - "подвисания"
int joystickReleasedWalk() {
  return sfAbs( svRemoteRightJoystickX ) < 5
      && sfAbs( svRemoteRightJoystickY ) < 5
      && sfAbs( svRemoteLeftJoystickX  ) < 5
      && !(svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1));
}

//1 - джойстик достаточно отклонен для продолжения ходьбы (гистерезис)
int joystickMovedWalk() {
  return sfAbs( svRemoteRightJoystickX ) >= 5
      || sfAbs( svRemoteRightJoystickY ) >= 5
      || sfAbs( svRemoteLeftJoystickX  ) >= 5;
}

void walk_params_to_fast() {
  fr1 = 2;
  fr2 = 6;
  gaitHeight = 170;
  amplitude = 2;
  stepHeight = 30;
}

void walk_params_to_normal() {
  fr1 = 8;
  fr2 = 12;
  gaitHeight = 165; // как в баскетболе
  amplitude = 32;
  stepHeight = 20;  // как в баскетболе
}

//Цикл ходьбы с пульта. Длина шага - правый джойстик Y
void walkLoop() {
  int i;
  //Первый шаг
  stepType = STEP_FIRST;
  stepLength = joystickToStepLength() / 3.0;
  sideLength = joystickToSideLength();
  selfFirstLegIsRightLeg = motion_to_right;
  forwardDirection += joystickToRotation() / 1000.0;
  stabilizeRotationByIMU();
  walkCycle(0);
  stepNumber -= 1;

  if( joystickReleasedWalk() && stepNumber < 1 )
    return;

  //Второй разгонный шаг
  stepType = STEP_OTHER;
  stepLength = joystickToStepLength() * 2.0 / 3.0;
  sideLength = joystickToSideLength();
  selfFirstLegIsRightLeg = motion_to_right;
  forwardDirection += joystickToRotation() / 1000.0;
  stabilizeRotationByIMU();
  walkCycle(0);
  stepNumber -= 1;

  if( joystickReleasedWalk() && stepNumber < 1 )
    return;

  //Одинаковые шаги, пока не бросят джойстик
  while( joystickMovedWalk() ) {
    stepLength = joystickToStepLength();
    sideLength = joystickToSideLength();
    forwardDirection += joystickToRotation() / 1000.0;
    stabilizeRotationByIMU();
    if( selfFirstLegIsRightLeg != motion_to_right ) walkCycle(1); // смена ноги при смене стороны
    selfFirstLegIsRightLeg = motion_to_right;
    walkCycle(0);
    stepNumber -= 1;
    if( (svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1)) || stepNumber > 0 ) {
      if( svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1) ) stepNumber = -svRemoteLeftJoystickY;
      for( i = 0; i < stepNumber; i++ ) {
        stabilizeRotationByIMU();
        walkCycle(0);
      }
    }
  }
}

//Быстрая ходьба с пульта (с зажатым L1)
void walkLoopFast() {
  int i;
  walk_params_to_fast();

  //Первый шаг
  stepType = STEP_FIRST;
  stepLength = joystickToStepLength() / 3.0;
  sideLength = joystickToSideLength();
  selfFirstLegIsRightLeg = motion_to_right;
  forwardDirection += joystickToRotation() / 1000.0;
  stabilizeRotationByIMU();
  walkCycle(0);
  stepNumber -= 1;

  if( joystickReleasedWalk() && stepNumber < 1 ) {
    walk_params_to_normal();
    return;
  }

  //Второй разгонный шаг
  stepType = STEP_OTHER;
  stepLength = joystickToStepLength() * 2.0 / 3.0;
  sideLength = joystickToSideLength();
  selfFirstLegIsRightLeg = motion_to_right;
  forwardDirection += joystickToRotation() / 1000.0;
  stabilizeRotationByIMU();
  walkCycle(0);
  stepNumber -= 1;

  if( joystickReleasedWalk() && stepNumber < 1 ) {
    walk_params_to_normal();
    return;
  }

  //Одинаковые шаги, пока не бросят джойстик
  while( joystickMovedWalk() ) {
    stepLength = joystickToStepLength();
    sideLength = joystickToSideLength();
    forwardDirection += joystickToRotation() / 1000.0;
    stabilizeRotationByIMU();
    if( selfFirstLegIsRightLeg != motion_to_right ) walkCycle(1);
    selfFirstLegIsRightLeg = motion_to_right;
    walkCycle(0);
    stepNumber -= 1;
    if( (svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1)) || stepNumber > 0 ) {
      if( svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1) ) stepNumber = -svRemoteLeftJoystickY;
      for( i = 0; i < stepNumber; i++ ) {
        stabilizeRotationByIMU();
        walkCycle(0);
      }
    }
  }
  walk_params_to_normal();
}

//Удар по мячу. kickByRight: 1 - правой, 0 - левой. invert: 1 - вперед, -1 - назад
void kick(int kickByRight, int small, int invert) {
  float gaitHeightKick  = 200;
  float stepHeightKick  = 50;
  float stepLengthKick  = 50 * invert;
  float kick_size = 100 * invert;
  float bodyTiltAtKick  = 0.03; // легкий наклон НАЗАД: был 0.04 вперед -
                                 // с инерцией замаха робот заваливался вперед
  int   pose_taking_cycles = 20;
  int   pose_hight_after_kick;
  float xrOld = xr, yrOld = yr;
  float xlOld = xl, ylOld = yl;

  leftRightFactor = 0; frontBackFactor = 0; // mixing мешает удару

  if( small == 1 ) kick_size = -10;
  selfFirstLegIsRightLeg = kickByRight;
  dx0Typical = stepLengthKick / (2 * fr1 + fr2 + 2.0 * framestep) * framestep;
  xr = xl = bodyTiltAtKick;
  yl =  solyLandingSkew;
  yr = -solyLandingSkew;
  s = (amplitude / 2) * sfMathCos(alpha01 * (fr1 / 2 + 0.5 * framestep));
  xtr = xtl = 0;

  //Начальная поза удара
  ztr = ztl = -gaitHeightKick;
  for( j = 0; j < pose_taking_cycles; j += 1 ) {
    ztr = ztr0 + j * (-ztr0 - gaitHeightKick) / pose_taking_cycles;
    ztl = ztr;
    ytr = -d10 - j * amplitude * 3 / pose_taking_cycles;
    ytl =  ytr + 2 * d10;
    computeAlphaForWalk();
  }

  ztr = ztl = -gaitHeightKick;
  ytr = s - d10;
  ytl = s + d10;

  //Замах и удар правой (переносящейся) ногой
  for( j = fr1; j < fr1 + fr2; j += framestep ) {
    ztr = -gaitHeightKick + stepHeightKick;
    dx  = stepLengthKick / fr2;
    dx0 = stepLengthKick / (2 * fr1 + fr2 + 4) * framestep;
    if( j == fr1 || j == fr1 + fr2 - 2 ) {
      xtr -= dx0;
      ytr = s - 64;
    }
    else
      xtr += dx * fr2 / (fr2 - 2 * framestep);
    if( j == fr1 + fr2 - 10 ) {
      xtr += kick_size;
      xtl -= kick_size;
    }
    if( j == fr1 + fr2 - 4 ) {
      xtr -= kick_size;
      xtl += kick_size;
    }
    xtl -= dx0;
    computeAlphaForWalk();
  }
  for( j = fr1 + fr2; j < 2 * fr1 + fr2; j += framestep ) {
    s = (amplitude / 2) * sfMathCos(alpha01 * (j - fr2 + framestep) / 2);
    ytr = s - d10;
    ytl = s + d10;
    ztl = -gaitHeightKick;
    ztr = -gaitHeightKick;
    dx0 = dx0Typical;
    xtl -= dx0;
    xtr -= dx0;
    computeAlphaForWalk();
  }
  for( j = 2 * fr1 + fr2; j < 2 * (fr1 + fr2); j += framestep ) {
    ztl = -gaitHeightKick + stepHeightKick;
    dx0 = dx0Typical * 4 / fr2;
    dx  = (stepLengthKick * (fr1 + fr2) / (4 * fr1) + 2 * dx0) / (fr2 - 2 * framestep) * framestep;
    if( j == (2 * fr1 + 2 * fr2 - framestep) ) {
      ztl = -gaitHeightKick;
      ytl = s + d10;
    }
    if( j == (2 * fr1 + fr2) || j == (2 * (fr1 + fr2) - framestep) ) {
      xtl -= dx0;
      ytl = s + 64;
    }
    else
      xtl += dx;
    xtr -= dx0;
    computeAlphaForWalk();
  }

  //Возврат в стойку после удара
  pose_taking_cycles = 2;
  pose_hight_after_kick = ztr0;
  for( j = 0; j < pose_taking_cycles; j += 1 ) {
    ztr = -gaitHeightKick + (j + 1) * (pose_hight_after_kick + gaitHeightKick) / pose_taking_cycles;
    ztl = ztr;
    ytr = -d10 - (pose_taking_cycles - (j + 1)) * amplitude / 2 / pose_taking_cycles;
    ytl =  d10 - (pose_taking_cycles - (j + 1)) * amplitude / 2 / pose_taking_cycles;
    computeAlphaForWalk();
  }
  pose_taking_cycles = 50;
  for( j = 0; j < pose_taking_cycles; j += 1 ) {
    ztr = pose_hight_after_kick + (j + 1) * (ztr0 - pose_hight_after_kick) / pose_taking_cycles;
    ztl = ztr;
    computeAlphaForWalk();
  }

  xr = xrOld;
  yr = yrOld;
  xl = xlOld;
  yl = ylOld;
  sfPoseGroup( MASK_ALL, 0, 200 );
  sfWaitFrame(200);
  sfWaitFrame(20); // дать стабилизоваться до включения mixing
  leftRightFactor = 100; frontBackFactor = 70;
}

//Поворот на месте. direction: 1 - по часовой, -1 - против
void turn(int direction, int factor) {
  int frames = 4;
  if( direction == 1 ) {
    sfPoseGroup( MASK_RIGHT_PELVIC, -200 * factor, frames );
    sfPoseGroup( MASK_LEFT_PELVIC,   200 * factor, frames );
  }
  if( direction == -1 ) {
    sfPoseGroup( MASK_RIGHT_PELVIC,  200 * factor, frames );
    sfPoseGroup( MASK_LEFT_PELVIC,  -200 * factor, frames );
  }
  sfPoseGroup( MASK_FOOT_SIDE, 200 * factor, frames );
  sfWaitFrame(frames);
  sfPoseGroup( MASK_RIGHT_PELVIC,     0, frames );
  sfPoseGroup( MASK_LEFT_PELVIC,      0, frames );
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE,  0, frames );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE,   0, frames );
  sfWaitFrame(frames);
  sfWaitFrame(6);
  forwardDirection = get_yaw();
}

void jump_forward(int factor) {
  leftRightFactor = 0; frontBackFactor = 0; // mixing мешает толчку
  int current_foot_front = (rightFootFrontTarget + leftFootFrontTarget) / 2;
  frameCount = 30;
  sfPoseGroup( MASK_FOOT_FRONT,  16 * factor + current_foot_front, frameCount );
  sfWaitFrame(frameCount);
  frameCount = 9;
  sfPoseGroup( MASK_FOOT_SIDE,   1000, frameCount );
  sfPoseGroup( MASK_FOOT_FRONT, -48 * factor + current_foot_front, frameCount );
  sfWaitFrame(frameCount);
  sfPoseGroup( MASK_FOOT_SIDE,   0, frameCount );
  sfPoseGroup( MASK_FOOT_FRONT,  16 * factor + current_foot_front, frameCount );
  sfWaitFrame(frameCount);
  sfPoseGroup( MASK_FOOT_FRONT,  0 + current_foot_front, frameCount );
  sfWaitFrame(frameCount);
  sfPoseGroup( MASK_ALL, 0, 100 );
  leftRightFactor = 100; frontBackFactor = 70;
}

void jump_backward(int factor) {
  leftRightFactor = 0; frontBackFactor = 0; // mixing мешает толчку
  int current_foot_front = (rightFootFrontTarget + leftFootFrontTarget) / 2;
  frameCount = 30;
  sfPoseGroup( MASK_FOOT_FRONT, -16 * factor + current_foot_front, frameCount );
  sfWaitFrame(frameCount);
  frameCount = 9;
  sfPoseGroup( MASK_FOOT_SIDE,   1000, frameCount );
  sfPoseGroup( MASK_FOOT_FRONT,  48 * factor + current_foot_front, frameCount );
  sfWaitFrame(frameCount);
  sfPoseGroup( MASK_FOOT_SIDE,   0, frameCount );
  sfPoseGroup( MASK_FOOT_FRONT, -16 * factor + current_foot_front, frameCount );
  sfWaitFrame(frameCount);
  sfPoseGroup( MASK_FOOT_FRONT,  0 + current_foot_front, frameCount );
  sfWaitFrame(frameCount);
  sfPoseGroup( MASK_ALL, 0, 100 );
  leftRightFactor = 100; frontBackFactor = 70;
}

void jump_left(int factor) {
  leftRightFactor = 0; frontBackFactor = 0; // mixing мешает толчку
  frameCount = 30;
  sfPoseGroup( MASK_LEFT_FOOT_SIDE,  -40 * factor, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE,  40 * factor, frameCount );
  sfPoseGroup( MASK_LEFT_HIP_SIDE,   -40 * factor, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE,   40 * factor, frameCount );
  //Наклон вперед в фазе приседания - компенсация заваливания назад
  //(вправо прыгает нормально, влево физически кидало назад).
  //Если станет заваливаться ВПЕРЕД - поменять знак на минус
  sfPoseGroup( MASK_FOOT_FRONT,       6 * factor, frameCount );
  sfWaitFrame(frameCount);
  frameCount = 9;
  sfPoseGroup( MASK_FOOT_SIDE,  200 * factor, frameCount ); // толчок влево сильнее:
  sfPoseGroup( MASK_FOOT_FRONT, 0, frameCount ); // часть энергии уходит на
  sfWaitFrame(frameCount);                         // компенсацию наклона - прыгал
  sfPoseGroup( MASK_FOOT_SIDE,       0, frameCount ); // слабее, чем вправо
  sfPoseGroup( MASK_LEFT_HIP_SIDE,   0, frameCount ); // вывернутой стопе
  sfPoseGroup( MASK_RIGHT_HIP_SIDE,  0, frameCount ); // и застревание в наклоне
  sfWaitFrame(frameCount);
  sfPoseGroup( MASK_ALL, 0, 100 );
  sfWaitFrame(20); // дать стабилизоваться до включения mixing
  leftRightFactor = 100; frontBackFactor = 70;
}

void jump_right(int factor) {
  leftRightFactor = 0; frontBackFactor = 0; // mixing мешает толчку
  frameCount = 30;
  sfPoseGroup( MASK_LEFT_FOOT_SIDE,   40 * factor, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, -40 * factor, frameCount );
  sfPoseGroup( MASK_LEFT_HIP_SIDE,    40 * factor, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE,  -40 * factor, frameCount );
  sfWaitFrame(frameCount);
  frameCount = 9;
  sfPoseGroup( MASK_FOOT_SIDE, 200 * factor, frameCount );
  sfWaitFrame(frameCount);
  sfPoseGroup( MASK_FOOT_SIDE,       0, frameCount );
  sfPoseGroup( MASK_LEFT_HIP_SIDE,   0, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE,  0, frameCount );
  sfWaitFrame(frameCount);
  //sfWaitFrame(frameCount);
  sfPoseGroup( MASK_ALL, 0, 100 );
  leftRightFactor = 100; frontBackFactor = 70;
}

//Шпагат. type: 1 - поперечный с возвратом, 2 - продольный
void splits(int type) {
  frameCount = 80;
  sfPoseGroup( MASK_PELVIC,     0,    frameCount );
  sfPoseGroup( MASK_HIP_SIDE,   0,    frameCount );
  sfPoseGroup( MASK_HIP,        3000, frameCount );
  sfPoseGroup( MASK_KNEE,       7200, frameCount );
  sfPoseGroup( MASK_FOOT_FRONT, 4100, frameCount );
  sfPoseGroup( MASK_FOOT_SIDE,  0,    frameCount );
  sfWaitFrame(frameCount);

  frameCount = 9;
  sfPoseGroup( MASK_FOOT_SIDE, 3000, frameCount );
  sfWaitFrame(frameCount);

  if( type == 1 ) {
    frameCount = 10;
    sfPoseGroup( MASK_PELVIC,     0,    frameCount );
    sfPoseGroup( MASK_HIP_SIDE,   1400, frameCount );
    sfPoseGroup( MASK_HIP,        3000, frameCount );
    sfPoseGroup( MASK_KNEE,       7200, frameCount );
    sfPoseGroup( MASK_FOOT_FRONT, 4100, frameCount );
    sfPoseGroup( MASK_FOOT_SIDE,  1400, frameCount );
    sfWaitFrame(frameCount);

    sfWaitFrame(100);

    frameCount = 9;
    sfPoseGroup( MASK_FOOT_SIDE, 3000, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 10;
    sfPoseGroup( MASK_HIP_SIDE,  0, frameCount );
    sfPoseGroup( MASK_FOOT_SIDE, 0, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 80;
    sfPoseGroup( MASK_ALL, 0, frameCount );
    sfWaitFrame(frameCount);
    return;
  }

  if( type == 2 ) {
    frameCount = 40;
    sfPoseGroup( MASK_HIP_SIDE,  4200, frameCount );
    sfPoseGroup( MASK_FOOT_SIDE, 1500, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 40;
    sfPoseGroup( MASK_HIP,        0, frameCount );
    sfPoseGroup( MASK_KNEE,       0, frameCount );
    sfPoseGroup( MASK_FOOT_FRONT, 0, frameCount );
    sfWaitFrame(frameCount);

    sfWaitFrame(400);

    frameCount = 80;
    sfPoseGroup( MASK_HIP,        3000, frameCount );
    sfPoseGroup( MASK_KNEE,       7200, frameCount );
    sfPoseGroup( MASK_FOOT_FRONT, 4100, frameCount );
    sfPoseGroup( MASK_FOOT_SIDE,  3600, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 10;
    sfPoseGroup( MASK_HIP_SIDE,  1400, frameCount );
    sfPoseGroup( MASK_FOOT_SIDE, 1400, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 9;
    sfPoseGroup( MASK_FOOT_SIDE, 3000, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 10;
    sfPoseGroup( MASK_HIP_SIDE,  0, frameCount );
    sfPoseGroup( MASK_FOOT_SIDE, 0, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 9;
    sfPoseGroup( MASK_FOOT_SIDE, 3000, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 80;
    sfPoseGroup( MASK_ALL, 0, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 9;
    sfPoseGroup( MASK_FOOT_SIDE, 3000, frameCount );
    sfWaitFrame(frameCount);

    frameCount = 9;
    sfPoseGroup( MASK_FOOT_SIDE, 0, frameCount );
    sfWaitFrame(frameCount);
  }
}

//Серия прыжков вперед (L1 + WALK_FAST)
void he(int r) {
  int i;
  for( i = 0; i < 35; i++ )
    jump_forward(5);
}

void movingRemoteControl() {
  while(1) {
    //Ожидание нажатия джойстика
    while( joystickReleased() ) {
      if( svRemoteButton == ARB_GETUP ) sfStartSlot( sfSlotIndex("standUpRemote.cpp") );
      if( svRemoteButton == ARB_JUMP )  sfStartSlot( sfSlotIndex("roki1p_Triple_Jump.cpp") );
      if( svRemoteButton == ARB_KICK_RIGHT || svRemoteButton == ARB_R2 ) kick(1, 0, 1);  // удар правой
      if( svRemoteButton == ARB_KICK_LEFT  || svRemoteButton == ARB_L2 ) kick(0, 0, 1);  // удар левой
      if( (svRemoteButton == HARD_BUTTON_L1 + ARB_R2) || (svRemoteButton == HARD_BUTTON_L1 + ARB_KICK_RIGHT) ) kick(1, 0, -1); // правой назад
      if( (svRemoteButton == HARD_BUTTON_R1 + ARB_L2) || (svRemoteButton == HARD_BUTTON_R1 + ARB_KICK_LEFT)  ) kick(0, 0, -1); // левой назад
      if( svRemoteButton == HARD_BUTTON_L1 + ARB_X ) jump_forward(10);
      if( svRemoteButton == HARD_BUTTON_L1 + ARB_B ) jump_backward(10);
      if( svRemoteButton == HARD_BUTTON_L1 + ARB_A ) jump_right(10);
      if( svRemoteButton == HARD_BUTTON_L1 + ARB_Y ) jump_left(10);
      if( svRemoteButton == HARD_BUTTON_R1 + ARB_X ) jump_forward(7);
      if( svRemoteButton == HARD_BUTTON_R1 + ARB_B ) jump_backward(7);
      if( svRemoteButton == HARD_BUTTON_R1 + ARB_A ) jump_right(7);
      if( svRemoteButton == HARD_BUTTON_R1 + ARB_Y ) jump_left(7);
      //Повороты: пока кнопка зажата - серия поворотов с паузой REPEAT_PAUSE
      if( svRemoteButton == ARB_A ) { turn(1, 10);  while( svRemoteButton == ARB_A ) { sfWaitFrame(REPEAT_PAUSE); turn(1, 10); } }   // по часовой
      if( svRemoteButton == ARB_Y ) { turn(-1, 10); while( svRemoteButton == ARB_Y ) { sfWaitFrame(REPEAT_PAUSE); turn(-1, 10); } }  // против часовой
      if( svRemoteButton == ARB_X ) { turn(1, 5);   while( svRemoteButton == ARB_X ) { sfWaitFrame(REPEAT_PAUSE); turn(1, 5); } }    // по часовой, слабее
      if( svRemoteButton == ARB_B ) { turn(-1, 5);  while( svRemoteButton == ARB_B ) { sfWaitFrame(REPEAT_PAUSE); turn(-1, 5); } }   // против часовой, слабее
      if( svRemoteButton == ARB_GETUP_FACEUP ) runTest();

      if( svRemoteButton == HARD_BUTTON_L1 + ARB_WALK_FAST ) he(10);
      if( svRemoteButton == HARD_BUTTON_L1 + ARB_WALK_SLOW ) sfStartSlot( sfSlotIndex("roki1p_Weight_Lifting1.cpp") );
      if( svRemoteButton == HARD_BUTTON_L1 + ARB_BEEP )      sfStartSlot( sfSlotIndex("roki1p_Basketball1.cpp") );
      if( svRemoteButton == HARD_BUTTON_L1 + ARB_JUMP )      sfStartSlot( sfSlotIndex("roki1p_Basketball0.cpp") );
      if( svRemoteButton == HARD_BUTTON_L1 )                 sfStartSlot( sfSlotIndex("roki1p_Triple_Jump_2.cpp") );

      if( svRemoteButton == ARB_BEEP ) {
        sfPoseGroup( MASK_ALL, 0, 100 );
        sfWaitFrame(100);
      }

      if( svRemoteButton == ARB_WALK_FAST ) {
        fps = 6;
        walking_frame = 1;
      }
      if( svRemoteButton == ARB_WALK_SLOW ) {
        fps = 8;
        walking_frame = 2;
      }
      if( svRemoteButton == ARB_SPLITS + HARD_BUTTON_R1 ) splits(2);
      sfWaitNextFrame();
    }

    //Джойстик нажали - переходим к циклу ходьбы
    sideLength = joystickToSideLength();
    selfFirstLegIsRightLeg = motion_to_right;
    if( svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1) ) stepNumber = -svRemoteLeftJoystickY;

    //Начальная поза
    walkInitialPoseFine();

    //Перезапоминаем направление "вперед" при каждом начале ходьбы -
    //иначе стабилизация тащит робота обратно к курсу, запомненному
    //при включении (после разворота/переноса робота он пытался вернуться)
    sfQuaternionToEulerImu();
    forwardDirection = get_yaw() - 0.02; // смещение курса, как в рабочем скрипте

    if( svRemoteButton == HARD_BUTTON_L1 ) walkLoopFast();
    else walkLoop();

    //Финальный шаг и исходная стойка
    stepType = STEP_LAST;
    walkCycle(0);
    walkFinalPoseFine();
  }
}

void main() {
  //РЕЖИМ ЗАПУСКА:
  // 0 - обычная работа с пультом ДУ
  // 1 - тест: без пульта идти вперед 20 шагов (проверка и подстройка)
  int testForward = 0;

  sfPoseGroup( MASK_ALL, 0, 100 );
  sfWaitFrame(100);

  slowWalk = 0;
  setup();
  sideLength = 0;
  rotation = 0;

  //Пауза 1 сек перед стартом. Робот должен стоять ровно и неподвижно -
  //в этот момент запоминается направление "вперед"
  sfWaitFrame(100);

  sfQuaternionToEulerImu();
  forwardDirection = get_yaw() - 0.02; // маленькое смещение курса, как в рабочем скрипте

  //Стабилизация крена по гироскопу - ВКЛЮЧЕНА, как в рабочем скрипте Баскетбол
  sfCreateTask( mixing, 20 );

  if( testForward == 1 ) {
    runTest();
    
  }
  else {
    movingRemoteControl();
  }
  sfStartSlot( sfSlotIndex("roki1TenseUp.cpp") );
}
