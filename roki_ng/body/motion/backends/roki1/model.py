#  Robot Data for Starkit Roki on RPI
#  Copyright STARKIT Soccer team of MIPT

class Robot:
    from roki_ng.vision.camera_geometry import CameraGeometry
    # Original HEIGHT_OF_CAMERA=413 mm, evaluated at reference head tilt.
    camera = CameraGeometry(height_m=.3985585334168535, forward_m=.028422652998)

    def __init__(self):

        self.model = 'Roki_1'
        self.ACTIVESERVOS = [
   #                             Real     Sim         min      max
   #    (Servo ID, Bus number,  FACTOR,  FACTOR,     Limit,   Limit)
            (10,        1,          1,      1,       -2462,    592),               # Прав Стопа Вбок
            (9,         1,         -1,      1,       -3619,    2462),              # Прав Стопа Вперед/Назад
            (8,         1,         -1,      1,       -4687,    949),               # Прав Колено
            (7,         1,         -1,     -1,       -2878,    3055),              # Прав Бедро Вперед/Назад
            (6,         1,         -1,      1,       -2666,    592),               # Прав Бедро Вбок
            (5,         1,         -1,      1,       -2666,    2666),              # Прав Таз Вращение
            (4,         1,         -1,      1,       -712,     2666),              # Прав Локоть Вперед/Назад
            (3,         1,         -1,      1,       -2666,    5332),              # Прав Локоть Вращение
            (2,         1,         -1,      1,       -5040,    386),               # Прав Плечо Вбок
            (1,         1,         -1,      1,       -6080,    2666),              # Прав Ключица
            (0,         2,          1,      1,       -2666,    2666),              # Торс
            (10,        2,          1,      1,       -592,     2462),              # Лев Стопа Вбок
            (9,         2,         -1,      1,       -2462,    3619),              # Лев Стопа Вперед/Назад
            (8,         2,         -1,      1,       -949,     4687),              # Лев Колено
            (7,         2,         -1,     -1,       -3055,    2878),              # Лев Бедро Вперед/Назад
            (6,         2,         -1,      1,       -592,     2666),              # Лев Бедро Вбок
            (5,         2,         -1,      1,       -2666,    2666),              # Лев Таз Вращение
            (4,         2,         -1,      1,       -2666,    712),               # Лев Локоть Вперед/Назад
            (3,         2,         -1,      1,       -5332,    2666),              # Лев Локоть Вращение
            (2,         2,         -1,      1,       -386,     5040),              # Лев Плечо Вбок
            (1,         2,         -1,      1,       -2666,    6080),              # Лев Ключица
            (0,         1,          1,      1,       -2666,    2666),              # Шея Вращение
            (12,        2,          1,      1,       -2600,    950),               # Шея Наклон
            #(11,        1,         -1,      1,       -8000,    8000),              # Прав Кисть
            #(11,        2,         -1,      1,       -8000,    8000)               # Лев Кисть
            ]

        self.jointLimits = []
        self.hand_servo_ids = [1,2,3,4]
        for servo in self.ACTIVESERVOS:
            self.jointLimits.append((servo[4], servo[5]))

        a5 = 25.2  # мм расстояние от оси симметрии до оси сервы 5
        b5 = 21.8  # мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
        c5 = 0     # мм расстояние от оси сервы 6 до нуля Z по вертикали
        a6 = 40    # мм расстояние от оси сервы 6 до оси сервы 7
        a7 = 64    # мм расстояние от оси сервы 7 до оси сервы 8
        a8 = 64    # мм расстояние от оси сервы 8 до оси сервы 9
        a9 = 40    # мм расстояние от оси сервы 9 до оси сервы 10
        a10= 12.5    # мм расстояние от оси сервы 10 до центра стопы по горизонтали
        b10= 24  # мм расстояние от оси сервы 10 до низа стопы   26.4
        c10 = 0    # мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
        self.e10 = 55 # мм половина длины стопы
        self.SIZES = [ a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 ]
        self.d10 = 59.5 #62 #53.4 # расстояние по Y от центра стопы до оси робота
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
        self.gaitHeight= 190         # Distance between Center of mass and floor in walk pose
        self.stepHeight = 32.0       # elevation of sole over floor
        self.LIMALPHA[3][1]=0
 #  end of  paramenetrs Not recommended for change

        self.ztr0 = -(c5+a6+a7+a8+a9+b10 -1) # -223.1
        self.ztl0 = -(c5+a6+a7+a8+a9+b10 -1) # -223.1

        self.ACTIVEJOINTS = ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5','hand_right_4',
            'hand_right_3','hand_right_2','hand_right_1','Tors1','Leg_left_10','Leg_left_9','Leg_left_8',
            'Leg_left_7','Leg_left_6','Leg_left_5','hand_left_4','hand_left_3','hand_left_2','hand_left_1','head0','head12', 'hand_right_11', 'hand_left_11']
        names = {(10, 1): 'MASK_RIGHT_FOOT_SIDE', (9, 1): 'MASK_RIGHT_FOOT_FRONT', (8, 1): 'MASK_RIGHT_KNEE', (7, 1): 'MASK_RIGHT_HIP', (6, 1): 'MASK_RIGHT_HIP_SIDE', (5, 1): 'MASK_RIGHT_PELVIC', (4, 1): 'MASK_RIGHT_ELBOW', (3, 1): 'MASK_RIGHT_ELBOW_SIDE', (2, 1): 'MASK_RIGHT_SHOULDER', (1, 1): 'MASK_RIGHT_CLAVICLE', (0, 2): 'MASK_TORSO_ROTATE', (10, 2): 'MASK_LEFT_FOOT_SIDE', (9, 2): 'MASK_LEFT_FOOT_FRONT', (8, 2): 'MASK_LEFT_KNEE', (7, 2): 'MASK_LEFT_HIP', (6, 2): 'MASK_LEFT_HIP_SIDE', (5, 2): 'MASK_LEFT_PELVIC', (4, 2): 'MASK_LEFT_ELBOW', (3, 2): 'MASK_LEFT_ELBOW_SIDE', (2, 2): 'MASK_LEFT_SHOULDER', (1, 2): 'MASK_LEFT_CLAVICLE', (0, 1): 'MASK_HEAD_ROTATE', (12, 2): 'MASK_HEAD_TILT', (11, 1): 'MASK_RIGHT_HAND', (11, 2): 'MASK_LEFT_HAND', (14, 1): 'MASK_RIGHT_CLAW', (14, 2): 'MASK_LEFT_CLAW'}
        self.ACTIVESERVOS = [tuple(row) + (names[row[0], row[1]],) for row in self.ACTIVESERVOS]
        self.hand_joints = [i for i, row in enumerate(self.ACTIVESERVOS) if row[0] in self.hand_servo_ids]
        from roki_ng.body.motion.mapping import JointMapping
        self.mapping = JointMapping(split_knee=False)
