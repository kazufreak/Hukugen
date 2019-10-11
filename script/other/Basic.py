import math
from decimal import *

def degconv_to10(deg):
    """角度度分秒00.0000値(60進）を10進へ変換"""
    a = float(Decimal(deg).quantize(Decimal('.01'), rounding=ROUND_DOWN))
    de = math.floor(deg)
    mi = (a - de) * 100 / 60
    se = (deg - a) * 1000 / 360

    return round(de + mi + se, 4)
