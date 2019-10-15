from Hukugen.script.other import Basic
import numpy as np

class jyouken:
    def __init__(self,L,Gl,Sl,Bcl,Gcal,W,deg,P,snowf,Qck,Qca,Ecu,Qsa1,Qsa2,Qsy,Es,Lgw,Rgw,
                 Lgt, Rgt,Lgp,Rgp,Syot,hs):
        """各種条件"""
        self.L = L   # 橋長
        self.Gl = Gl  # 桁長
        self.Sl = Sl  # 斜め支間長
        self.Bcl = Bcl  # 支障中心間隔
        self.Gcal = Gcal  # 計算支間長
        self.W = W  # 幅員(車道部)
        self.deg = Basic.degconv_to10(deg)  # 斜角 60進から10進へ変換
        self.P = P  # 活荷重kN
        self.snowf = snowf  # 雪荷重
        self.Qck = Qck  # Co設計基準強度
        self.Qca = Qca  # Co許容曲げ圧縮応力度
        self.Ecu = Ecu  # Co終局ひずみ
        self.Qsa1 = Qsa1  # 鉄筋引張応力度（死荷時）
        self.Qsa2 = Qsa2  # 鉄筋引張応力度（活荷時）
        self.Qsy = Qsy  # 引張鉄筋の降伏点
        self.Es = Es  # ヤング係数
        """寸法条件"""
        self.Lgw = Lgw  # 左地覆幅
        self.Rgw = Rgw  # 右地覆幅
        self.Lgt = Lgt  # 左地覆厚
        self.Rgt = Rgt  # 右地覆厚
        self.Lgp = Lgp  # 左高欄重量　ｋN/m2
        self.Rgp = Rgp  # 右高欄重量　kN/m2
        self.Syot = Syot  # 床版厚
        self.hs = hs  # 舗装厚mm

    def ForceCondi(self):
        """死荷重"""
        Dgp = self.Lgp + self.Rgp  # 高欄重量　ｋN/m2
        Dp = self.Syot * 24.5 + self.Lgt * 24.5 + self.Rgt * 24.5  # 主版部　ｋN/m2
        """活荷重"""
        Qgp = (1.1 + self.Syot/2) * 2.5  # 高欄推力による曲げモーメント
        i = 20 / (50 + self.Sl)  # 衝撃係数
        return Dgp, Dp, Qgp, i

    def EikyouPa(self):
        """影響宣パラメータ"""
        lb = self.Sl / (self.W + self.Lgw + self.Rgw)  # 支間長と主版幅の比
        c2 = 200 + 2 * self.hs
        c2_ = 500 + 2 * self.hs
        cb = c2 * 10**-3 / (2 * (self.W + self.Lgw + self.Rgw))
        return lb,c2,c2_,cb

    def olzen(self):
        lb, c2, c2_, cb = self.EikyouPa() # パラメータ読込
        """オルゼン図表計算1.00～1.50までの対応"""
        olzen_100 = [
            [0.000, 0.461, 0.316, 0.234, 0.179, 0.140, 0.112, 0.092, 0.076],
            [0.350, 0.331, 0.268, 0.209, 0.163, 0.129, 0.103, 0.085, 0.070],
            [0.193, 0.192, 0.175, 0.149, 0.120, 0.097, 0.079, 0.065, 0.054],
            [0.089, 0.090, 0.085, 0.075, 0.063, 0.051, 0.043, 0.035, 0.029]
        ]

        olzen_125 = [
            [0.000, 0.524, 0.381, 0.299, 0.244, 0.203, 0.174, 0.150, 0.130],
            [0.364, 0.354, 0.310, 0.261, 0.219, 0.184, 0.159, 0.138, 0.121],
            [0.206, 0.203, 0.198, 0.179, 0.157, 0.138, 0.121, 0.106, 0.093],
            [0.094, 0.096, 0.094, 0.090, 0.081, 0.071, 0.064, 0.058, 0.051]
        ]

        olzen_150 = [
            [0.000, 0.587, 0.444, 0.363, 0.308, 0.267, 0.237, 0.211, 0.189],
            [0.384, 0.380, 0.350, 0.311, 0.273, 0.241, 0.216, 0.193, 0.175],
            [0.223, 0.222, 0.220, 0.208, 0.192, 0.177, 0.161, 0.147, 0.134],
            [0.103, 0.105, 0.105, 0.104, 0.098, 0.091, 0.085, 0.079, 0.073]
        ]

        olzen5_100 = [
            [0.179, 0.197, 0.224, 0.277, 0.000],
            [0.163, 0.176, 0.192, 0.203, 0.190],
            [0.120, 0.127, 0.130, 0.124, 0.117],
            [0.063, 0.065, 0.064, 0.060, 0.057]
        ]

        olzen5_125 = [
            [0.245, 0.260, 0.287, 0.339, 0.000],
            [0.219, 0.229, 0.239, 0.242, 0.230],
            [0.158, 0.161, 0.160, 0.152, 0.147],
            [0.081, 0.081, 0.079, 0.075, 0.073]
        ]

        olzen5_150 = [
            [0.308, 0.323, 0.349, 0.400, 0.000],
            [0.273, 0.281, 0.286, 0.284, 0.272],
            [0.193, 0.193, 0.190, 0.182, 0.179],
            [0.098, 0.097, 0.095, 0.091, 0.090]
        ]
        Emx = np.zeros((4, 9), float)  # a1_計算_Mx影響値
        Emx5 = np.zeros((4, 5), float)  # a5_計算_Mx影響値

        # a1における計算
        if lb >= 1.25 and lb <= 1.50:
            for i in range(len(olzen_100)):
                for n in range(len(olzen_100[0])):
                    value = round(olzen_125[i][n] + (olzen_150[i][n] - olzen_125[i][n]) / 0.25 * (lb - 1.25), 3)
                    Emx[i][n] = value

        if lb >= 1.00 and lb <= 1.25:
            for i in range(len(olzen_100)):
                for n in range(len(olzen_100[0])):
                    value = round(olzen_100[i][n] + (olzen_125[i][n] - olzen_100[i][n]) / 0.25 * (lb - 1.00), 3)
                    Emx[i][n] = value

        # a5における計算
        if lb >= 1.25 and lb <= 1.50:
            for i in range(len(olzen5_100)):
                for n in range(len(olzen5_100[0])):
                    value = round(olzen5_125[i][n] + (olzen5_150[i][n] - olzen5_125[i][n]) / 0.25 * (lb - 1.25), 3)
                    Emx5[i][n] = value

        if lb >= 1.00 and lb <= 1.25:
            for i in range(len(olzen5_100)):
                for n in range(len(olzen5_100[0])):
                    value = round(olzen5_100[i][n] + (olzen5_125[i][n] - olzen5_100[i][n]) / 0.25 * (lb - 1.00), 3)
                    Emx5[i][n] = value
        # 尖角値
        sentan_100 = {0.015: 1.020, 0.02: 0.950, 0.03: 0.870, 0.04: 0.810, 0.06: 0.720, 0.08: 0.660}
        sentan_125 = {0.015: 1.080, 0.02: 1.020, 0.03: 0.930, 0.04: 0.870, 0.06: 0.780, 0.08: 0.720}
        sentan_150 = {0.015: 1.140, 0.02: 1.090, 0.03: 1.000, 0.04: 0.930, 0.06: 0.850, 0.08: 0.780}

        sentan5_100 = {0.015: 0.450, 0.02: 0.420, 0.03: 0.390, 0.04: 0.370, 0.06: 0.330, 0.08: 0.310}
        sentan5_125 = {0.015: 0.510, 0.02: 0.480, 0.03: 0.450, 0.04: 0.430, 0.06: 0.400, 0.08: 0.370}
        sentan5_150 = {0.015: 0.570, 0.02: 0.550, 0.03: 0.510, 0.04: 0.490, 0.06: 0.460, 0.08: 0.430}

        Stn = {}  # a1_計算_尖角値
        Stn5 = {}  # a5_計算_尖角値

        if lb >= 1.25 and lb <= 1.50:
            key = [i for i in sentan_100.keys()]
            for i in key:
                value = round(sentan_125[i] + (sentan_150[i] - sentan_125[i]) / 0.25 * (lb - 1.25), 3)
                Stn[i] = value

        if lb >= 1.00 and lb <= 1.25:
            key = [i for i in sentan_100.keys()]
            for i in key:
                value = round(sentan_100[i] + (sentan_125[i] - sentan_100[i]) / 0.25 * (lb - 1.00), 3)
                Stn[i] = value

        if lb >= 1.25 and lb <= 1.50:
            key = [i for i in sentan5_100.keys()]
            for i in key:
                value = round(sentan5_125[i] + (sentan5_150[i] - sentan5_125[i]) / 0.25 * (lb - 1.25), 3)
                Stn5[i] = value

        if lb >= 1.00 and lb <= 1.25:
            key = [i for i in sentan5_100.keys()]
            for i in key:
                value = round(sentan5_100[i] + (sentan5_125[i] - sentan5_100[i]) / 0.25 * (lb - 1.00), 3)
                Stn5[i] = value

        Stcb = {}  # a1_対象計算_補間値
        Stcb5 = {}  # a5_対象計算_補間値

        ke = [i for i in Stn.keys()]
        count = 0
        for i in ke:
            count += 1
            if cb >= i and cb <= ke[count]:
                kou = (Stn[i] - Stn[ke[count]]) / (ke[count] - i)
                dl = cb - i
                value = round(Stn[i] - kou * dl, 3)
                Stcb[cb] = value


        ke5 = [i for i in Stn5.keys()]
        count5 = 0
        for i in ke:
            count5 += 1
            if cb >= i and cb <= ke5[count5]:
                kou = (Stn5[i] - Stn5[ke5[count5]]) / (ke5[count5] - i)
                dl = cb - i
                value = round(Stn5[i] - kou * dl, 3)
                Stcb5[cb] = value

        # 縁端荷重による係数
        kei100 = [0.253, 0.110, 0.048, 0.326, -0.062, -0.078]
        kei125 = [0.263, 0.147, 0.083, 0.317, -0.054, -0.117]
        kei150 = [0.277, 0.181, 0.119, 0.305, -0.044, -0.149]

        Kei = np.zeros(6, float)  # 計算_縁端荷重係数

        if lb >= 1.25 and lb <= 1.50:
            for i in range(len(kei100)):
                    value = round(kei125[i] + (kei150[i] - kei125[i]) / 0.25 * (lb - 1.25), 3)
                    Kei[i] = value
        if lb >= 1.00 and lb <= 1.25:
            for i in range(len(kei100)):
                value = round(kei100[i] + (kei125[i] - kei100[i]) / 0.25 * (lb - 1.00), 3)
                Kei[i] = value


        print("a1")
        print(Emx)
        print(Stn)
        print(Stcb)

        print("a5")
        print(Emx5)
        print(Stn5)
        print(Stcb5)

        print("縁端荷重による係数")
        print(Kei)
        return Emx, Stn, Stcb, Emx5, Stn5, Stcb5, Kei

    def aMoment(self):
        """影響面積計算"""
        mainBL = round((self.W + self.Lgw + self.Rgw) / 8, 3)




    def calctest(self):
        Dgp, Dp, Qgp, i = self.ForceCondi()
        lb, c2, c2_, cb = self.EikyouPa()

        Pr = Dgp + Qgp
        # Pr･l･βi + mr･γi


def olzen(lb, cb):
    """オルゼン図表値"""
    olzen_100 = [
        [0.000, 0.461, 0.316, 0.234, 0.179, 0.140, 0.112, 0.092, 0.076],
        [0.350, 0.331, 0.268, 0.209, 0.163, 0.129, 0.103, 0.085, 0.070],
        [0.193, 0.192, 0.175, 0.149, 0.120, 0.097, 0.079, 0.065, 0.054],
        [0.089, 0.090, 0.085, 0.075, 0.063, 0.051, 0.043, 0.035, 0.029]
    ]

    olzen_125 = [
        [0.000, 0.524, 0.381, 0.299, 0.244, 0.203, 0.174, 0.150, 0.130],
        [0.364, 0.354, 0.310, 0.261, 0.219, 0.184, 0.159, 0.138, 0.121],
        [0.206, 0.203, 0.198, 0.179, 0.157, 0.138, 0.121, 0.106, 0.093],
        [0.094, 0.096, 0.094, 0.090, 0.081, 0.071, 0.064, 0.058, 0.051]
    ]

    olzen_150 = [
        [0.000, 0.587, 0.444, 0.363, 0.308, 0.267, 0.237, 0.211, 0.189],
        [0.384, 0.380, 0.350, 0.311, 0.273, 0.241, 0.216, 0.193, 0.175],
        [0.223, 0.222, 0.220, 0.208, 0.192, 0.177, 0.161, 0.147, 0.134],
        [0.103, 0.105, 0.105, 0.104, 0.098, 0.091, 0.085, 0.079, 0.073]
    ]

    olzen5_100 = [
        [0.179, 0.197, 0.224, 0.277, 0.000],
        [0.163, 0.176, 0.192, 0.203, 0.190],
        [0.120, 0.127, 0.130, 0.124, 0.117],
        [0.063, 0.065, 0.064, 0.060, 0.057]
    ]

    olzen5_125 = [
        [0.245, 0.260, 0.287, 0.339, 0.000],
        [0.219, 0.229, 0.239, 0.242, 0.230],
        [0.158, 0.161, 0.160, 0.152, 0.147],
        [0.081, 0.081, 0.079, 0.075, 0.073]
    ]

    olzen5_150 = [
        [0.308, 0.323, 0.349, 0.400, 0.000],
        [0.273, 0.281, 0.286, 0.284, 0.272],
        [0.193, 0.193, 0.190, 0.182, 0.179],
        [0.098, 0.097, 0.095, 0.091, 0.090]
    ]
    Emx = np.zeros((4, 9), float)  # a1_計算_Mx影響値
    Emx5 = np.zeros((4, 5), float)  # a5_計算_Mx影響値

    # a1における計算
    if lb >= 1.25 and lb <= 1.50:
        for i in range(len(olzen_100)):
            for n in range(len(olzen_100[0])):
                value = round(olzen_125[i][n] + (olzen_150[i][n] - olzen_125[i][n]) / 0.25 * (lb - 1.25), 3)
                Emx[i][n] = value

    if lb >= 1.00 and lb <= 1.25:
        for i in range(len(olzen_100)):
            for n in range(len(olzen_100[0])):
                value = round(olzen_100[i][n] + (olzen_125[i][n] - olzen_100[i][n]) / 0.25 * (lb - 1.00), 3)
                Emx[i][n] = value

    # a5における計算
    if lb >= 1.25 and lb <= 1.50:
        for i in range(len(olzen5_100)):
            for n in range(len(olzen5_100[0])):
                value = round(olzen5_125[i][n] + (olzen5_150[i][n] - olzen5_125[i][n]) / 0.25 * (lb - 1.25), 3)
                Emx5[i][n] = value

    if lb >= 1.00 and lb <= 1.25:
        for i in range(len(olzen5_100)):
            for n in range(len(olzen5_100[0])):
                value = round(olzen5_100[i][n] + (olzen5_125[i][n] - olzen5_100[i][n]) / 0.25 * (lb - 1.00), 3)
                Emx5[i][n] = value
    # 尖角値
    sentan_100 = {0.015: 1.020, 0.02: 0.950, 0.03: 0.870, 0.04: 0.810, 0.06: 0.720, 0.08: 0.660}
    sentan_125 = {0.015: 1.080, 0.02: 1.020, 0.03: 0.930, 0.04: 0.870, 0.06: 0.780, 0.08: 0.720}
    sentan_150 = {0.015: 1.140, 0.02: 1.090, 0.03: 1.000, 0.04: 0.930, 0.06: 0.850, 0.08: 0.780}

    sentan5_100 = {0.015: 0.450, 0.02: 0.420, 0.03: 0.390, 0.04: 0.370, 0.06: 0.330, 0.08: 0.310}
    sentan5_125 = {0.015: 0.510, 0.02: 0.480, 0.03: 0.450, 0.04: 0.430, 0.06: 0.400, 0.08: 0.370}
    sentan5_150 = {0.015: 0.570, 0.02: 0.550, 0.03: 0.510, 0.04: 0.490, 0.06: 0.460, 0.08: 0.430}

    Stn = {}  # a1_計算_尖角値
    Stn5 = {}  # a5_計算_尖角値

    if lb >= 1.25 and lb <= 1.50:
        key = [i for i in sentan_100.keys()]
        for i in key:
            value = round(sentan_125[i] + (sentan_150[i] - sentan_125[i]) / 0.25 * (lb - 1.25), 3)
            Stn[i] = value

    if lb >= 1.00 and lb <= 1.25:
        key = [i for i in sentan_100.keys()]
        for i in key:
            value = round(sentan_100[i] + (sentan_125[i] - sentan_100[i]) / 0.25 * (lb - 1.00), 3)
            Stn[i] = value

    if lb >= 1.25 and lb <= 1.50:
        key = [i for i in sentan5_100.keys()]
        for i in key:
            value = round(sentan5_125[i] + (sentan5_150[i] - sentan5_125[i]) / 0.25 * (lb - 1.25), 3)
            Stn5[i] = value

    if lb >= 1.00 and lb <= 1.25:
        key = [i for i in sentan5_100.keys()]
        for i in key:
            value = round(sentan5_100[i] + (sentan5_125[i] - sentan5_100[i]) / 0.25 * (lb - 1.00), 3)
            Stn5[i] = value

    Stcb = {}  # a1_対象計算_補間値
    Stcb5 = {}  # a5_対象計算_補間値

    ke = [i for i in Stn.keys()]
    count = 0
    for i in ke:
        count += 1
        if cb >= i and cb <= ke[count]:
            kou = (Stn[i] - Stn[ke[count]]) / (ke[count] - i)
            dl = cb - i
            value = round(Stn[i] - kou * dl, 3)
            Stcb[cb] = value


    ke5 = [i for i in Stn5.keys()]
    count5 = 0
    for i in ke:
        count5 += 1
        if cb >= i and cb <= ke5[count5]:
            kou = (Stn5[i] - Stn5[ke5[count5]]) / (ke5[count5] - i)
            dl = cb - i
            value = round(Stn5[i] - kou * dl, 3)
            Stcb5[cb] = value

    # 縁端荷重による係数
    kei100 = [0.253, 0.110, 0.048, 0.326, -0.062, -0.078]
    kei125 = [0.263, 0.147, 0.083, 0.317, -0.054, -0.117]
    kei150 = [0.277, 0.181, 0.119, 0.305, -0.044, -0.149]

    Kei = np.zeros(6, float)  # 計算_縁端荷重係数

    if lb >= 1.25 and lb <= 1.50:
        for i in range(len(kei100)):
                value = round(kei125[i] + (kei150[i] - kei125[i]) / 0.25 * (lb - 1.25), 3)
                Kei[i] = value
    if lb >= 1.00 and lb <= 1.25:
        for i in range(len(kei100)):
            value = round(kei100[i] + (kei125[i] - kei100[i]) / 0.25 * (lb - 1.00), 3)
            Kei[i] = value


    print("a1")
    print(Emx)
    print(Stn)
    print(Stcb)

    print("a5")
    print(Emx5)
    print(Stn5)
    print(Stcb5)

    print("縁端荷重による係数")
    print(Kei)

    return Emx, Stn, Stcb, Emx5, Stn5, Stcb5, Kei

def EikyouArea(no, W, Stcb, Emx, Stn):
    # no:区間ナンバー L:区間延長
    no = no








Emx, Stn, Stcb, Emx5, Stn5, Stcb5, Kei = olzen(1.30, 0.031)




