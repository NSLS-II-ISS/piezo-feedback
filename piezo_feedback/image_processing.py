
import numpy as np
from scipy.optimize import curve_fit

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
    return not_saturated and not_empty

def analyze_image(image,
                  line = 420,
                  center=600,
                  n_lines = 1,
                  truncate_data=True,
                  should_print_diagnostics=True):

    beam_profile = reduce_image(image, line, n_lines)
    profile_is_good = check_image_quality(beam_profile, n_lines)

    if profile_is_good:
        try:
            npts = beam_profile.size
            x = np.arange(0, npts)[::-1]
            center = npts - beam_profile.argmax()
            if truncate_data:
                idx_to_fit = np.where(beam_profile > beam_profile.max() / 2)
                coeff, var_matrix = curve_fit(gauss, x[idx_to_fit], beam_profile[idx_to_fit], p0=[1, center, 5])
            else:
                coeff, var_matrix = curve_fit(gauss, x, beam_profile, p0=[1, center, 5])
            return coeff[1]
        except:
            if should_print_diagnostics:
                print('>>>> FEEDBACK - failed - Fitting failure')
            return None
    else:
        if should_print_diagnostics:
            print('>>>> FEEDBACK - failed - image is either empty or saturated')
        return None