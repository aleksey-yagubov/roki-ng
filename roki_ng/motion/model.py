#  Robot Data for Starkit Roki on RPI
#  Copyright STARKIT Soccer team of MIPT

class Robot:

    def __init__(self):

        self.model = 'Roki_2'
        self.ACTIVESERVOS = [
   #                             Real     Sim         min      max                                 Kondo
   #    (Servo ID, Bus number,  FACTOR,  FACTOR,     Limit,   Limit,                               2Roki)
            (10,        1,          1,     -1,        -711,    711,     'MASK_RIGHT_FOOT_SIDE',       -1),               # Прав Стопа Вбок
            (9,         1,         -1,      1,       -1333,    2310,    'MASK_RIGHT_FOOT_FRONT',       1),              # Прав Стопа Вперед/Назад
            (8,         1,         -1,     -1,       -5333,    237,     'MASK_RIGHT_KNEE',            -1),               # Прав Колено
            (7,         1,         -1,     -1,       -3851,    1481,    'MASK_RIGHT_HIP',             -1),              # Прав Бедро Вперед/Назад
            (6,         1,         -1,      1,       -3199,    444,     'MASK_RIGHT_HIP_SIDE',        -1),               # Прав Бедро Вбок
            (5,         1,         -1,      1,       -2666,    2666,    'MASK_RIGHT_PELVIC',          -1),              # Прав Таз Вращение
            (4,         1,         -1,      1,       -711,     3850,    'MASK_RIGHT_ELBOW',            1),              # Прав Локоть Вперед/Назад
            (3,         1,         -1,      1,       -3555,    5332,    'MASK_RIGHT_ELBOW_SIDE',      -1),              # Прав Локоть Вращение
            (2,         1,         -1,      1,       -5036,    680,     'MASK_RIGHT_SHOULDER',        -1),               # Прав Плечо Вбок
            (1,         1,         -1,      1,       -5925,    3555,    'MASK_RIGHT_CLAVICLE',        -1),              # Прав Ключица
            (0,         2,          1,      1,       -2666,    2666,    'MASK_TORSO_ROTATE',          -1),              # Торс
            (10,        2,          1,     -1,       -711,     711,     'MASK_LEFT_FOOT_SIDE',         1),              # Лев Стопа Вбок
            (9,         2,         -1,     -1,       -2310,    1333,    'MASK_LEFT_FOOT_FRONT',       -1),              # Лев Стопа Вперед/Назад
            (8,         2,         -1,      1,       -237,     5333,    'MASK_LEFT_KNEE',              1),              # Лев Колено
            (7,         2,         -1,     -1,       -1481,    3851,    'MASK_LEFT_HIP',               1),              # Лев Бедро Вперед/Назад
            (6,         2,         -1,      1,       -444,     3199,    'MASK_LEFT_HIP_SIDE',          1),              # Лев Бедро Вбок
            (5,         2,         -1,      1,       -2666,    2666,    'MASK_LEFT_PELVIC',            1),              # Лев Таз Вращение
            (4,         2,         -1,      1,       -3850,    711,     'MASK_LEFT_ELBOW',            -1),               # Лев Локоть Вперед/Назад
            (3,         2,         -1,      1,       -5332,    3555,    'MASK_LEFT_ELBOW_SIDE',        1),              # Лев Локоть Вращение
            (2,         2,         -1,      1,       -680,     5036,    'MASK_LEFT_SHOULDER',          1),              # Лев Плечо Вбок
            (1,         2,         -1,      1,       -3555,    5925,    'MASK_LEFT_CLAVICLE',          1),              # Лев Ключица
            (0,         1,          1,      1,       -2666,    2666,    'MASK_HEAD_ROTATE',           -1),              # Шея Вращение
            (12,        2,          1,      1,       -2600,    950,     'MASK_HEAD_TILT',             -1),               # Шея Наклон
            (11,        1,          1,      1,       -2667,    5333,    'MASK_RIGHT_HAND',             1),              # Прав Кисть
            (11,        2,          1,      1,       -2667,    5333,    'MASK_LEFT_HAND',              1),               # Лев Кисть
            (14,        1,          1,      1,           0,    2281,    'MASK_RIGHT_CLAW',             1),              # Прав Кисть
            (14,        2,          1,      1,           0,    2281,    'MASK_LEFT_CLAW',              1)               # Лев Кисть
            ]
        self.hand_servo_ids = [1,2,3,4,11,14]
        self.jointLimits = []
        for servo in self.ACTIVESERVOS:
            self.jointLimits.append((servo[4], servo[5]))

        a5 = 40.2  # мм расстояние от оси симметрии до оси сервы 5
        b5 = 0     # мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
        c5 = 0     # мм расстояние от оси сервы 6 до нуля Z по вертикали
        a6 = 0    # мм расстояние от оси сервы 6 до оси сервы 7
        a7 = 99    # мм расстояние от оси сервы 7 до оси сервы 8
        a8 = 99    # мм расстояние от оси сервы 8 до оси сервы 9
        a9 = 0     # мм расстояние от оси сервы 9 до оси сервы 10
        a10= 13.7    # мм расстояние от оси сервы 10 до центра стопы по горизонтали
        b10= 23.8  # мм расстояние от оси сервы 10 до низа стопы   26.4
        c10 = 0    # мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
        self.e10 = 55 # мм половина длины стопы
        self.SIZES = [ a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 ]
        self.d10 = 62 #62 #53.4 # расстояние по Y от центра стопы до оси робота
        limAlpha5 = [self.ACTIVESERVOS[5][4], self.ACTIVESERVOS[5][5]]
        limAlpha6 = [self.ACTIVESERVOS[4][4], self.ACTIVESERVOS[4][5]]
        limAlpha7 = [self.ACTIVESERVOS[3][4], self.ACTIVESERVOS[3][5]]
        limAlpha8 = [self.ACTIVESERVOS[2][4], self.ACTIVESERVOS[2][5]]
        limAlpha9 = [self.ACTIVESERVOS[1][4], self.ACTIVESERVOS[1][5]]
        limAlpha10 = [self.ACTIVESERVOS[0][4], self.ACTIVESERVOS[0][5]]
        self.LIMALPHA = [limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10]

# Following paramenetrs Not recommended for change
        self.amplitude = 32          # mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
        self.fr1 =8                  # frame number for 1-st phase of gait ( two legs on floor)
        self.fr2 = 12                # frame number for 2-nd phase of gait ( one leg in air)
        self.gaitHeight= 180         # Distance between Center of mass and floor in walk pose
        self.stepHeight = 40 #32.0       # elevation of sole over floor
        self.LIMALPHA[3][1]=0
 #  end of  paramenetrs Not recommended for change

        self.ztr0 = -(c5+a6+a7+a8+a9+b10 -1) # -220.8
        self.ztl0 = -(c5+a6+a7+a8+a9+b10 -1) # -220.8

        self.ACTIVEJOINTS = ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5','hand_right_4',
            'hand_right_3','hand_right_2','hand_right_1','Tors1','Leg_left_10','Leg_left_9','Leg_left_8',
            'Leg_left_7','Leg_left_6','Leg_left_5','hand_left_4','hand_left_3','hand_left_2','hand_left_1'
            ,'head0','head12', 'hand_right_11', 'hand_left_11', 'hand_right_14', 'hand_left_14']

        self.hand_joints =[6, 7, 8, 9, 17, 18, 19, 20, 23, 24, 25, 26]
