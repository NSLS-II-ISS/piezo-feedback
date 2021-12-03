
import numpy as np
from scipy.optimize import curve_fit
import time as ttime

def print_msg_now(msg):
    print(f'*({datetime.strftime(datetime.now(), "%Y-%m-%d %H:%M:%S.%f")}) {msg}')

def gauss(x, *p):
    A, mu, sigma = p
    return A * np.exp(-(x - mu) ** 2 / (2. * sigma ** 2))

def reduce_image(image, line, n_lines):
    # the old way:
    # sum_lines = sum(image[:, [i for i in range(int(line - np.floor(n_lines/2)),
    #                                            int(line + np.ceil(n_lines/2)))]].transpose())

    idx_lo = int(line - np.floor(n_lines / 2))
    idx_hi = int(line + np.ceil(n_lines / 2))
    beam_profile = np.sum(image[:, idx_lo: idx_hi], axis=1)

    if len(beam_profile) > 0:
        beam_profile = beam_profile - np.mean(beam_profile[:200])  # empirically we determined that first 200 pixels are BKG

    return beam_profile

def check_image_quality(beam_profile, n_lines):
    min_value = beam_profile.min()
    max_value = beam_profile.max()
    not_saturated = (max_value <= n_lines * 100)
    not_empty = (max_value >= 10) and (((max_value - min_value) / n_lines) > 5)
    if not_saturated and not_empty:
        return 'good'
    if not_saturated:
        return 'empty'
    if not_empty:
        return 'saturated'

# def check_image_quality(image, line, n_lines): WIP
#     idx_lo = int(line - np.floor(n_lines / 2))
#     idx_hi = int(line + np.ceil(n_lines / 2))
#     beam_profile = np.mean(image[:, idx_lo: idx_hi], axis=1)
#     max_value = beam_profile.max()
#     min_value = beam_profile.min()
#
#     not_saturated = max_value <= 150
#     not_empty = max_value >= 7
#     if not_saturated and not_empty:
#         return 'good'
#     if not_saturated:
#         return 'empty'
#     if not_empty:
#         return 'saturated'

def analyze_image(image,
                  line = 420,
                  center=600,
                  n_lines = 1,
                  truncate_data=True,
                  should_print_diagnostics=True):

    beam_profile = reduce_image(image, line, n_lines)
    image_quality = check_image_quality(beam_profile, n_lines)
    # image_quality = check_image_quality(image, line, n_lines)

    err_msg = ''
    if image_quality == 'good':
        try:
            npts = beam_profile.size
            x = np.arange(0, npts)[::-1]
            center = npts - beam_profile.argmax()
            beam_profile /= beam_profile.max()
            if truncate_data:
                idx_to_fit = np.where(beam_profile > beam_profile.max() / 2)
                coeff, var_matrix = curve_fit(gauss, x[idx_to_fit], beam_profile[idx_to_fit], p0=[1, center, 40])
            else:
                coeff, var_matrix = curve_fit(gauss, x, beam_profile, p0=[1, center, 40])
            err_msg = ''
            return coeff[1], err_msg
        except:
            err_msg = 'fitting'
            if should_print_diagnostics:
                print_msg_now(f'Feedback error: fitting failure')
            # return None
    else:
        err_msg = f'{image_quality} image'
        if should_print_diagnostics:
            print_msg_now(f'Feedback error: image is either empty or saturated')
    return None, err_msg