from xas.pid import PID
import time as ttime
import numpy as np
import sys

_args = sys.argv
if len(_args) > 1: # if ran as a script with PATH
    PATH = _args[1]
else: # if imported as a module
    PATH = ''
    from piezo_feedback.image_processing import analyze_image



class PiezoFeedback:

    def __init__(self, hhm, bpm_es, shutters, sample_time = 0.01, host='remote'):

        self.hhm = hhm
        self.bpm_es = bpm_es
        self.shutters = shutters
        self.host = host

        P = 0.004 * 1
        I = 0  # 0.02
        D = 0  # 0.01
        self.pid = PID(P, I, D)
        self.pid.windup_guard = 3
        self.pid.setSampleTime(sample_time)

        self.image_size_x = self.bpm_es.cam.array_size.array_size_x.get()
        self.image_size_y = self.bpm_es.cam.array_size.array_size_y.get()

        self.go = 0
        self.should_print_diagnostics = True
        self.truncate_data = False

        self.read_fb_parameters()
        self.subscribe_fb_parameters()

        self.read_shutter_status()
        self.subscribe_shutter_status()


    def set_fb_parameters(self, line, center, n_lines, n_measures, pcoeff):
        self.hhm.fb_line.put(line)
        self.hhm.fb_center.put(center)
        self.hhm.fb_nlines.put(n_lines)
        self.hhm.fb_nmeasures.put(n_measures)
        self.hhm.fb_pcoeff.put(pcoeff)
        # self.read_piezo_analysis_parameters()

    def read_fb_parameters(self):
        self.line = int(self.hhm.fb_line.get())
        self.center = float(self.hhm.fb_center.get())
        self.pid.SetPoint = self.center
        self.n_lines = int(self.hhm.fb_nlines.get())
        self.n_measures = int(self.hhm.fb_nmeasures.get())
        self.pid.Kp = float(0.004 * self.hhm.fb_pcoeff.get())
        self.status = int(self.hhm.fb_status.get())

    def subscribe_fb_parameters(self):
        def update_fb_kp(value, old_value, **kwargs):
            self.pid.Kp = 0.004 * float(value)

        def update_fb_nmeasures(value, old_value, **kwargs):
            self.n_measures = int(value)

        def update_fb_nlines(value, old_value, **kwargs):
            self.n_lines = int(value)

        def update_fb_center(value, old_value, **kwargs):
            self.center = float(value)
            self.pid.SetPoint = self.center  # why invert???

        def update_fb_line(value, old_value, **kwargs):
            self.line = int(value)

        def update_fb_status(value, old_value, **kwargs):
            self.status = int(value)

        self.hhm.fb_pcoeff.subscribe(update_fb_kp)
        self.hhm.fb_nmeasures.subscribe(update_fb_nmeasures)
        self.hhm.fb_nlines.subscribe(update_fb_nlines)
        self.hhm.fb_center.subscribe(update_fb_center)
        self.hhm.fb_line.subscribe(update_fb_line)
        self.hhm.fb_status.subscribe(update_fb_status)


    def read_shutter_status(self):
        self.fe_open = (self.shutters['FE Shutter'].state.get() == 0)
        self.ph_open = (self.shutters['PH Shutter'].state.get() == 0)


    def subscribe_shutter_status(self):
        def update_fe_shutter(value, old_value, **kwargs):
            if value == 0 and old_value == 1:
                self.fe_open = True
            elif value == 1 and old_value == 0:
                self.fe_open = False

        def update_ph_shutter(value, old_value, **kwargs):
            if value == 0 and old_value == 1:
                self.ph_open = True
            elif value == 1 and old_value == 0:
                self.ph_open = False

        self.shutters['FE Shutter'].state.subscribe(update_fe_shutter)
        self.shutters['PH Shutter'].state.subscribe(update_ph_shutter)


    def take_image(self):
        try:
            image = self.bpm_es.image.array_data.read()['bpm_es_image_array_data']['value'].reshape((960,1280))
            image = image.astype(np.int16)
        except Exception as e:
            print(f"Exception: {e}\nPlease, check the max retries value in the piezo feedback IOC or maybe the network load (too many cameras).")
            image = None

        return image


    def find_beam_position(self):
        image = self.take_image()
        if image is None: return
        beam_position = analyze_image(image,
                                      line=self.line,
                                      center=self.center,
                                      n_lines=self.n_lines,
                                      truncate_data=self.truncate_data,
                                      should_print_diagnostics=self.should_print_diagnostics)
        return beam_position


    def update_center(self):
        centers = []
        for i in range(self.n_measures):
            current_position = self.find_beam_position()
            if current_position is not None:
                centers.append(current_position)

        if len(centers) > 0:
            center_av = np.mean(centers)
            self.hhm.fb_center.put(center_av) # this should automatically update the self.center and self.pid.SetPoint due to subscription


    def adjust_pitch(self):
        center_rb = self.find_beam_position()

        if center_rb is not None:
            self.pid.update(center_rb)
            pitch_delta = self.pid.output
            pitch_current = self.hhm.pitch.user_readback.get()
            pitch_target = pitch_current + pitch_delta

            try:
                if pitch_target > 100:
                    self.hhm.pitch.move(pitch_target)
                self.should_print_diagnostics = True
            except:
                if self.should_print_diagnostics:
                    self.should_print_diagnostics = False
        else:
            self.should_print_diagnostics = False


    @property
    def shutters_open(self):
        return (self.fe_open and self.ph_open)


    @property
    def fb_status(self):
        if self.host == 'remote':
            return self.status
        elif self.host == 'local':
            return self.go


    def run(self):
        while 1:
            if self.fb_status and self.shutters_open:
                self.adjust_pitch()
                ttime.sleep(self.pid.sample_time)
            else:
                ttime.sleep(0.25)



if __name__ == "__main__":
    exec(open(PATH + 'mini_profile.py').read())
    exec(open(PATH + 'image_processing.py').read())

    piezo_feedback = PiezoFeedback(hhm, bpm_es, shutters, host='remote')

    piezo_feedback.run()



