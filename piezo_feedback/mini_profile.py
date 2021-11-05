
from ophyd import (EpicsMotor, Device, Component as Cpt,
                   EpicsSignal)
from ophyd import (ProsilicaDetector, SingleTrigger,
                   EpicsSignalRO, ImagePlugin, StatsPlugin, ROIPlugin,
                   DeviceStatus)


class HHM(Device):
    pitch = Cpt(EpicsMotor, 'Mono:HHM-Ax:P}Mtr')

    fb_status = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-Sts')
    fb_center = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-Center')
    fb_line = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-Line')
    fb_nlines = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-NLines')
    fb_nmeasures = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-NMeasures')
    fb_pcoeff = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-PCoeff')
    fb_hostname = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-Hostname')
    fb_heartbeat = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-Heartbeat')
    fb_status_err = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-Err')
    fb_status_msg = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-StsMsg', string=True)

hhm = HHM('XF:08IDA-OP{', name='hhm')



class BPM(ProsilicaDetector, SingleTrigger):
    image = Cpt(ImagePlugin, 'image1:')
    stats1 = Cpt(StatsPlugin, 'Stats1:')
    stats2 = Cpt(StatsPlugin, 'Stats2:')
    roi1 = Cpt(ROIPlugin, 'ROI1:')
    roi2 = Cpt(ROIPlugin, 'ROI2:')
    counts = Cpt(EpicsSignal, 'Pos:Counts')
    acquire = Cpt(EpicsSignal, 'cam1:Acquire')
    # Dan Allan guessed about the nature of these signals. Fix them if you need them.
    ins = Cpt(EpicsSignal, 'Cmd:In-Cmd')
    ret = Cpt(EpicsSignal, 'Cmd:Out-Cmd')
    switch_insert = Cpt(EpicsSignalRO, 'Sw:InLim-Sts')
    switch_retract = Cpt(EpicsSignalRO, 'Sw:OutLim-Sts')
    polarity = 'pos'

    def insert(self):
        self.ins.put(1)

    def retract(self):
        self.ret.put(1)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stage_sigs.clear()  # default stage sigs do not apply

    @property
    def acquiring(self):
        return bool(self.acquire.get())

    def append_ioc_reboot_pv(self, ioc_reboot_pv):
        self.ioc_reboot_pv = ioc_reboot_pv

    def reboot_ioc(self):
        self.ioc_reboot_pv.put(1)
        ttime.sleep(5)
        self.acquire.put(1)



bpm_es = BPM('XF:08IDB-BI{BPM:ES}', name='bpm_es')
bpm_es_ioc_reset = EpicsSignal('XF:08IDB-CT{IOC:BPM:ES}:SysReset', name='bpm_es_ioc_reset')
bpm_es.append_ioc_reboot_pv(bpm_es_ioc_reset)


class EPS_Shutter(Device):
    state = Cpt(EpicsSignal, 'Pos-Sts')
    cls = Cpt(EpicsSignal, 'Cmd:Cls-Cmd')
    opn = Cpt(EpicsSignal, 'Cmd:Opn-Cmd')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.color = 'red'
        # self.shutter_type = shutter_type

shutter_fe = EPS_Shutter('XF:08ID-PPS{Sh:FE}', name = 'FE Shutter')
shutter_ph = EPS_Shutter('XF:08IDA-PPS{PSh}', name = 'PH Shutter')

while not shutter_ph.connected or not shutter_fe.connected:
    ttime.sleep(0.1)

shutters = {shutter_fe.name: shutter_fe,
            shutter_ph.name: shutter_ph}