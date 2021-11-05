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

    def __init__(self, hhm, bpm_es, shutters, sample_time = 0.01, local_hostname='remote'):

        self.hhm = hhm
        self.bpm_es = bpm_es
        self.shutters = shutters
        self.local_hostname = local_hostname

        P = 0.004 * 1
        I = 0  # 0.02
        D = 0  # 0.01
        self.pid = PID(P, I, D)
        self.pid.windup_guard = 3
        self.pid.setSampleTime(sample_time)

        self.image_size_x = self.bpm_es.cam.array_size.array_size_x.get()
        self.image_size_y = self.bpm_es.cam.array_size.array_size_y.get()

        # self.go = 0
        self.should_print_diagnostics = True
        self.truncate_data = False

        self.read_fb_parameters()
        self.subscribe_fb_parameters()

        self.read_shutter_status()
        self.subscribe_shutter_status()

        # self._fb_step_start = 0
        self._hb_step_start = None # heartbeat timer
        self.previous_image = None

        #
        # self._n_max_data = 500
        # self._pitch_vals = np.zeros(self._n_max_data)
        # self._centers = np.zeros(self._n_max_data)
        # self._timestamps = np.zeros(self._n_max_data)
        # self._idx = 0



    def set_fb_parameters(self, center, line, n_lines, n_measures, pcoeff, host):
        self.hhm.fb_center.put(center)
        self.hhm.fb_line.put(line)
        self.hhm.fb_nlines.put(n_lines)
        self.hhm.fb_nmeasures.put(n_measures)
        self.hhm.fb_pcoeff.put(pcoeff)
        self.hhm.fb_hostname.put(host)


    def read_fb_parameters(self):
        self.line = int(self.hhm.fb_line.get())
        self.center = float(self.hhm.fb_center.get())
        self.pid.SetPoint = self.center
        self.n_lines = int(self.hhm.fb_nlines.get())
        self.n_measures = int(self.hhm.fb_nmeasures.get())
        self.pcoeff = self.hhm.fb_pcoeff.get()
        self.pid.Kp = float(0.004 * self.pcoeff)
        self.status = bool(self.hhm.fb_status.get())
        self.host = str(self.hhm.fb_hostname.get())

    def current_fb_parameters(self):
        return (self.center, self.line, self.n_lines, self.n_measures, self.pcoeff, self.host)

    def subscribe_fb_parameters(self):
        def update_fb_kp(value, old_value, **kwargs):
            self.pcoeff = float(value)
            self.pid.Kp = 0.004 * float(value)

        def update_fb_nmeasures(value, old_value, **kwargs):
            self.n_measures = int(value)

        def update_fb_nlines(value, old_value, **kwargs):
            self.n_lines = int(value)

        def update_fb_center(value, old_value, **kwargs):
            self.center = float(value)
            self.pid.SetPoint = self.center

        def update_fb_line(value, old_value, **kwargs):
            self.line = int(value)

        def update_fb_status(value, old_value, **kwargs):
            self.status = bool(value)

        def update_host(value, old_value, **kwargs):
            self.host = str(value)

        self.hhm.fb_pcoeff.subscribe(update_fb_kp)
        self.hhm.fb_nmeasures.subscribe(update_fb_nmeasures)
        self.hhm.fb_nlines.subscribe(update_fb_nlines)
        self.hhm.fb_center.subscribe(update_fb_center)
        self.hhm.fb_line.subscribe(update_fb_line)
        self.hhm.fb_status.subscribe(update_fb_status)
        self.hhm.fb_hostname.subscribe(update_host)

    def tweak_fb_center(self, shift=1):
        cur_value = self.center
        self.hhm.fb_center.put(cur_value + shift)


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

    def check_image(self, image):
        err_msg = ''
        if (self.previous_image is not None) and (self.bpm_es.acquiring):
            if np.all(image == self.previous_image):
                image = None
                err_msg = 'ioc freeze'
                self.report_fb_error(err_msg)
                self.bpm_es.reboot_ioc()
        if (self.previous_image is None):
            self.previous_image = image.copy()
        return image, err_msg

    def take_image(self):
        try:
            image = self.bpm_es.image.array_data.read()['bpm_es_image_array_data']['value'].reshape((960,1280))
            image = image.astype(np.int16)
            image, err_msg = self.check_image(image)

        except Exception as e:
            print(f'{ttime.ctime()} Exception: {e}\nPlease, check the max retries value in the piezo feedback IOC or maybe the network load (too many cameras).')
            image, err_msg = None, 'network'
        return None, err_msg

    def find_beam_position(self):
        image, err_msg = self.take_image()
        if image is not None:
            beam_position, err_msg = analyze_image(image,
                                                   line=self.line,
                                                   center=self.center,
                                                   n_lines=self.n_lines,
                                                   truncate_data=self.truncate_data)
            return beam_position, err_msg
        else:
            return None, err_msg

    def update_center(self):
        centers = []
        for i in range(self.n_measures):
            current_position, err_msg = self.find_beam_position()
            if current_position is not None:
                centers.append(current_position)

        if len(centers) > 0:
            center_av = np.mean(centers)
            self.hhm.fb_center.put(center_av) # this should automatically update the self.center and self.pid.SetPoint due to subscription
            self.report_no_fb_error()
        else:
            self.report_fb_error(err_msg)


    def adjust_pitch(self):
        # print('attempting to adjust pitch', end= ' ... ')
        center_rb, err_msg = self.find_beam_position()
        adjustment_success = False
        if center_rb is not None:
            self.pid.update(center_rb)
            pitch_delta = self.pid.output
            pitch_current = self.hhm.pitch.user_readback.get()
            pitch_target = pitch_current + pitch_delta
            try:
                if pitch_target > 100:
                    self.hhm.pitch.move(pitch_target)
                self.should_print_diagnostics = True
                adjustment_success = True
                self.report_no_fb_error()
            except:
                self.should_print_diagnostics = False
                self.report_fb_error(err_msg)
        else:
            self.should_print_diagnostics = False
            self.report_fb_error(err_msg)
        return adjustment_success

    def report_fb_error(self, err_msg):
        self.hhm.fb_status_err.put(1)
        self.hhm.fb_status_msg.put(err_msg)

    def report_no_fb_error(self):
        self.hhm.fb_status_err.put(0)
        self.hhm.fb_status_msg.put('')


    # def update_deviation_data(self, timestamp, center, pitch):
    #     self._timestamps = self._update_finite_array(self._timestamps, timestamp, self._idx, self._n_max_data)
    #     self._centers = self._update_finite_array(self._centers, center, self._idx, self._n_max_data)
    #     self._pitch_vals = self._update_finite_array(self._pitch_vals, pitch, self._idx, self._n_max_data)
    #     if self._idx < self._n_max_data:
    #         self._idx += 1
    #
    #
    # def _update_finite_array(self, x, new_value, idx, n_max):
    #     if idx < n_max:
    #         x[idx] = new_value
    #     else:
    #         x[:-1] = x[1:]
    #         x[-1] = new_value
    #     return x



    @property
    def shutters_open(self):
        return (self.fe_open and self.ph_open)
        # return (self.ph_open)

    @property
    def feedback_on(self):
        return self.status

    @property
    def local_hosting(self):
        return (self.local_hostname == self.host)

    @property
    def status_err(self):
        return self.hhm.fb_status_err.get()

    @property
    def status_msg(self):
        return self.hhm.fb_status_msg.get()

    def _start_timers(self):
        # self._fb_step_start = ttime.time()
        if self._hb_step_start is None:
            self._hb_step_start = ttime.time()


    def emit_heartbeat_signal(self, thresh=0.75):
        elapsed_time = ttime.time() - self._hb_step_start
        if elapsed_time > thresh:
            if self.hhm.fb_heartbeat.get() == 0:
                self.hhm.fb_heartbeat.put(1)
            else:
                self.hhm.fb_heartbeat.put(0)
            self._hb_step_start = None



    def run(self):
        while 1:
            if self.local_hosting:
                self._start_timers()
                if self.feedback_on and self.shutters_open:
                    adjustment_success = self.adjust_pitch()
                    if adjustment_success:
                        ttime.sleep(self.pid.sample_time)
                    else:
                        ttime.sleep(0.25)
                else:
                    ttime.sleep(0.25)
                self.emit_heartbeat_signal()
            else:
                ttime.sleep(1)







if __name__ == "__main__":
    exec(open(PATH + 'mini_profile.py').read())
    exec(open(PATH + 'image_processing.py').read())
    piezo_feedback = PiezoFeedback(hhm, bpm_es, shutters, local_hostname='remote')
    piezo_feedback.run()



