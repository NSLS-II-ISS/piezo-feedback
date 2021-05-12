from ophyd import setup_ophyd
setup_ophyd()
from ophyd import (EpicsMotor, Device, Component as Cpt,
                   EpicsSignal)
from ophyd import (ProsilicaDetector, SingleTrigger,
                   EpicsSignalRO, ImagePlugin, StatsPlugin, ROIPlugin,
                   DeviceStatus)
#from isstools.pid import PID
from xas.pid import PID
import time as ttime
import math
from scipy.optimize import curve_fit

import numpy as np


class HHM(Device):
    pitch = Cpt(EpicsMotor, 'Mono:HHM-Ax:P}Mtr')

    fb_status = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-Sts')
    fb_center = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-Center')
    fb_line = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-Line')
    fb_nlines = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-NLines')
    fb_nmeasures = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-NMeasures')
    fb_pcoeff = Cpt(EpicsSignal, 'Mono:HHM-Ax:P}FB-PCoeff')

hhm = HHM('XF:08IDA-OP{', name='hhm')


class BPM(ProsilicaDetector, SingleTrigger):
    image = Cpt(ImagePlugin, 'image1:')
    stats1 = Cpt(StatsPlugin, 'Stats1:')
    stats2 = Cpt(StatsPlugin, 'Stats2:')
    roi1 = Cpt(ROIPlugin, 'ROI1:')
    roi2 = Cpt(ROIPlugin, 'ROI2:')
    counts = Cpt(EpicsSignal, 'Pos:Counts')
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

bpm_es = BPM('XF:08IDB-BI{BPM:ES}', name='bpm_es')


class EPS_Shutter(Device):
    state = Cpt(EpicsSignal, 'Pos-Sts')
    cls = Cpt(EpicsSignal, 'Cmd:Cls-Cmd')
    opn = Cpt(EpicsSignal, 'Cmd:Opn-Cmd')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.color = 'red'

shutter_fe = EPS_Shutter('XF:08ID-PPS{Sh:FE}', name = 'FE Shutter')
shutter_fe.shutter_type = 'FE'
shutter_ph = EPS_Shutter('XF:08IDA-PPS{PSh}', name = 'PH Shutter')
shutter_ph.shutter_type = 'PH'

while not shutter_ph.connected or not shutter_fe.connected:
    ttime.sleep(0.1)

shutters = {shutter_fe.name: shutter_fe,
            shutter_ph.name: shutter_ph}

piezo_kp = float(hhm.fb_pcoeff.get())
piezo_nmeasures = int(hhm.fb_nmeasures.get())
piezo_nlines = int(hhm.fb_nlines.get())
piezo_center = float(hhm.fb_center.get())
piezo_line = int(hhm.fb_line.get())
piezo_fb_status = int(hhm.fb_status.get())

P = 0.004 * piezo_kp
I = 0
D = 0

pid = PID(P, I, D)
sampleTime = 0.00025
pid.setSampleTime(sampleTime)
pid.windup_guard = 3

def update_fb_kp(value, old_value, **kwargs):
    global piezo_kp
    piezo_kp = float(value)
    pid.Kp = 0.004 * piezo_kp

def update_fb_nmeasures(value, old_value, **kwargs):
    global piezo_nmeasures
    piezo_nmeasures = int(value)

def update_fb_nlines(value, old_value, **kwargs):
    global piezo_nlines
    piezo_nlines = int(value)

def update_fb_center(value, old_value, **kwargs):
    global piezo_center
    piezo_center = float(value)

def update_fb_line(value, old_value, **kwargs):
    global piezo_line
    piezo_line = int(value)

def update_fb_status(value, old_value, **kwargs):
    global piezo_fb_status
    piezo_fb_status = int(value) 

hhm.fb_pcoeff.subscribe(update_fb_kp)
hhm.fb_nmeasures.subscribe(update_fb_nmeasures)
hhm.fb_nlines.subscribe(update_fb_nlines)
hhm.fb_center.subscribe(update_fb_center)
hhm.fb_line.subscribe(update_fb_line)
hhm.fb_status.subscribe(update_fb_status)


def gauss(x, *p):
    A, mu, sigma = p
    return A * np.exp(-(x - mu) ** 2 / (2. * sigma ** 2))

def gaussian_piezo_feedback(line = 420, center_point = 655, 
                            n_lines = 1, n_measures = 10):

    image = bpm_es.image.array_data.read()['bpm_es_image_array_data']['value'].reshape((960,1280))

    image = image.astype(np.int16)
    sum_lines = sum(image[:, [i for i in range(line - math.floor(n_lines/2), line + math.ceil(n_lines/2))]].transpose())
    if len(sum_lines) > 0:
        sum_lines = sum_lines - (sum(sum_lines) / len(sum_lines))
    index_max = sum_lines.argmax()
    max_value = sum_lines.max()
    min_value = sum_lines.min()


    if max_value >= 10 and max_value <= n_lines * 100 and ((max_value - min_value) / n_lines) > 5:
        coeff, var_matrix = curve_fit(gauss, list(range(960)), sum_lines, p0=[1, index_max, 5])
        pid.SetPoint = 960 - center_point
        pid.update(coeff[1])
        deviation = pid.output
        piezo_diff = deviation

        curr_value = hhm.pitch.read()['hhm_pitch']['value']
        hhm.pitch.move(curr_value - piezo_diff)

def run():
    while 1:
        if piezo_fb_status and len([shutters[shutter] for shutter in shutters if
                shutters[shutter].shutter_type != 'SP' and
                shutters[shutter].state.read()['{}_state'.format(shutter)][
                             'value'] != 0]) == 0:
            gaussian_piezo_feedback(line=piezo_line,
                                         center_point=piezo_center,
                                         n_lines=piezo_nlines,
                                         n_measures=piezo_nmeasures)
            ttime.sleep(sampleTime)
        else:
            ttime.sleep(sampleTime)

if __name__ == "__main__":
    run()
