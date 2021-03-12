__author__ = 'Geir Istad'
"""
The MIT License (MIT)

SimplePID - PID in Python made easy!
Copyright (c) 2015 Geir Istad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import utime


class SimplePID():
    __last_time_ms = 0.0
    __delta_time_ms = 0.0
    __kp = 0.0
    __ki = 0.0
    __kd = 0.0
    __set_point = 0.0
    __output_value_min_1 = 0.0
    __error_min_1 = 0.0
    __error_min_2 = 0.0
    __coeff_a = 0.0
    __coeff_b = 0.0
    __coeff_c = 0.0
    __min_output = 0.0
    __max_output = 0.0
    __PID_direction_direct = True

    def __init__(self, a_set_point, a_min_output, a_max_output, a_kp, a_ki,
                 a_kd, a_delta_time_ms=100, a_direction_direct=True):
        self.__last_time_ms = utime.ticks_ms()
        self.__set_point = a_set_point
        self.__min_output = a_min_output
        self.__max_output = a_max_output
        self.__kp = a_kp
        self.__ki = a_ki
        self.__kd = a_kd
        self.__delta_time_ms = a_delta_time_ms
        self.__PID_direction_direct = a_direction_direct
        self.__update_coeffs()

    def get_controller_direction(self):
        return self.__PID_direction_direct

    def set_controller_direction(self, a_direction_direct):
        self.__PID_direction_direct = a_direction_direct
        self.__update_coeffs()

    def get_delta_time_ms(self):
        return self.__delta_time

    def set_delta_time_ms(self, a_delta_time_ms):
        self.__delta_time_ms = a_delta_time_ms
        self.__update_coeffs()

    def get_kp(self):
        return self.__kp

    def get_ki(self):
        return self.__ki

    def get_kd(self):
        return self.__kd

    def set_kp(self, a_kp):
        self.__kp = a_kp
        self.__update_coeffs()

    def set_ki(self, a_ki):
        self.__ki = a_ki
        self.__update_coeffs()

    def set_kd(self, a_kd):
        self.__kd = a_kd
        self.__update_coeffs()

    def check_time(self):
        current_time_ms = 1000*(utime.ticks_ms())
        if current_time_ms - self.__last_time_ms > self.__delta_time_ms:
            self.__last_time_ms = current_time_ms
            return True
        else:
            return False

    def get_output_value(self, a_feedback_value):
        current_error = self.__set_point - a_feedback_value
        factor_1 = self.__output_value_min_1
        factor_2 = self.__coeff_a * current_error
        factor_3 = self.__coeff_b * self.__error_min_1
        factor_4 = self.__coeff_c * self.__error_min_2
        output_value = factor_1 + factor_2 + factor_3 + factor_4

        if output_value > self.__max_output:
            output_value = self.__max_output
        elif output_value < self.__min_output:
            output_value = self.__min_output

        self.__error_min_2 = self.__error_min_1
        self.__error_min_1 = current_error
        self.__output_value_min_1 = output_value

        return output_value

    def __update_coeffs(self):
        self.__update_coeff_a()
        self.__update_coeff_b()
        self.__update_coeff_c()

    def __update_coeff_a(self):
        if self.__PID_direction_direct is True:
            kp = float(self.__kp)
            ki = float(self.__ki)
            kd = float(self.__kd)
        else:
            kp = 0.0 - self.__kp
            ki = 0.0 - self.__ki
            kd = 0.0 - self.__kd
        factor_1 = kp
        factor_2 = ki * (self.__delta_time_ms / (1000.0 * 2.0))
        factor_3 = kd / (self.__delta_time_ms / 1000.0)
        self.__coeff_a = factor_1 + factor_2 + factor_3

    def __update_coeff_b(self):
        if self.__PID_direction_direct is True:
            kp = float(self.__kp)
            ki = float(self.__ki)
            kd = float(self.__kd)
        else:
            kp = 0.0 - self.__kp
            ki = 0.0 - self.__ki
            kd = 0.0 - self.__kd
        factor_1 = - kp
        factor_2 = ki * (self.__delta_time_ms / (1000.0 * 2.0))
        factor_3 = - ((2.0 * kd) / (self.__delta_time_ms / 1000.0))
        self.__coeff_b = factor_1 + factor_2 + factor_3

    def __update_coeff_c(self):
        if self.__PID_direction_direct is True:
            kd = self.__kd
        else:
            kd = 0.0 - self.__kd
        self.__coeff_c = kd / (self.__delta_time_ms / 1000.0)

