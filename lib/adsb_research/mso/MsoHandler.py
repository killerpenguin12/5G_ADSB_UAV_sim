from mso.UAT import UAT
import mso.propagation_param as p

class MsoHandler:
    def __init__(self) -> None:
        self.uat = UAT(0,0)
        self.msos = []

    def addMsos(self, lat, lon, alt):
        uat = UAT(0,0)
        msos = []
        for i in range(len(lat)):
            uat.extract_lsbs((lat[i], lon[i], alt[i]))
            msos.append(uat.full_mso_range())
        return msos

    def getMso(self, coord):
        self.uat.extract_lsbs((coord[0], coord[1], coord[2]))
        return self.uat.full_mso_range()
