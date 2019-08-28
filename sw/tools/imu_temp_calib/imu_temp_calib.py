#!/usr/bin/env python3
import argparse
import pandas
import re
import numpy as np


def imu_temp_calib(data_files, ac_id=None, decimals=0, plot_results=False, **kwargs):
    data_gyro = []
    data_accel = []
    for file in data_files:
        print('Reading {}...'.format(file))
        with open(file, 'rt') as f:
            for line in f:
                m_gyro = re.match(
                    r'(?P<timestamp>[0-9.]+) (?P<ac_id>[0-9]+) IMU_GYRO_TEMP (?P<sender_id>[0-9]+) (?P<temp>[0-9.]+) (?P<p_raw>[0-9\-]+) (?P<q_raw>[0-9\-]+) (?P<r_raw>[0-9\-]+) (?P<p_neutral>[0-9\-]+) (?P<q_neutral>[0-9\-]+) (?P<r_neutral>[0-9\-]+)',
                    line)
                m_accel = re.match(
                    r'(?P<timestamp>[0-9.]+) (?P<ac_id>[0-9]+) IMU_ACCEL_TEMP (?P<sender_id>[0-9]+) (?P<temp>[0-9.]+) (?P<x_raw>[0-9\-]+) (?P<y_raw>[0-9\-]+) (?P<z_raw>[0-9\-]+) (?P<x_neutral>[0-9\-]+) (?P<y_neutral>[0-9\-]+) (?P<z_neutral>[0-9\-]+)',
                    line)
                if m_gyro:
                    if not ac_id or ac_id == int(m_gyro.group('ac_id')):
                        print(line)
                        data_gyro.append({
                            'file': file,
                            'timestamp': float(m_gyro.group('timestamp')),
                            'ac_id': int(m_gyro.group('ac_id')),
                            'sender_id': int(m_gyro.group('sender_id')),
                            'temp': float(m_gyro.group('temp')),
                            'p_raw': int(m_gyro.group('p_raw')),
                            'q_raw': int(m_gyro.group('q_raw')),
                            'r_raw': int(m_gyro.group('r_raw')),
                            'p_neutral': int(m_gyro.group('p_neutral')),
                            'q_neutral': int(m_gyro.group('q_neutral')),
                            'r_neutral': int(m_gyro.group('r_neutral'))
                    })
                if m_accel:
                    if not ac_id or ac_id == int(m_accel.group('ac_id')):
                        print(line)
                        data_accel.append({
                            'file': file,
                            'timestamp': float(m_accel.group('timestamp')),
                            'ac_id': int(m_accel.group('ac_id')),
                            'sender_id': int(m_accel.group('sender_id')),
                            'temp': float(m_accel.group('temp')),
                            'x_raw': int(m_accel.group('x_raw')),
                            'y_raw': int(m_accel.group('y_raw')),
                            'z_raw': int(m_accel.group('z_raw')),
                            'x_neutral': int(m_accel.group('x_neutral')),
                            'y_neutral': int(m_accel.group('y_neutral')),
                            'z_neutral': int(m_accel.group('z_neutral'))
                        })
    df_gyro = pandas.DataFrame(data_gyro)
    df_accel = pandas.DataFrame(data_accel)

    # Find and neutral values (incl. gravity)
    neutral = [
        np.mean(df_gyro['p_raw']),
        np.mean(df_gyro['q_raw']),
        np.mean(df_gyro['r_raw']),
        np.mean(df_accel['x_raw']),
        np.mean(df_accel['y_raw']),
        np.mean(df_accel['z_raw'])
    ]

    # Generate LUT
    df_gyro_lut = df_gyro.copy()
    df_gyro_lut['temp'] = np.floor(df_gyro_lut['temp'])
    df_gyro_lut = pandas.pivot_table(df_gyro_lut, values=['p_raw', 'q_raw', 'r_raw'], index='temp', aggfunc=np.mean)
    df_gyro_lut = df_gyro_lut - np.array(neutral[0:3])
    df_accel_lut = df_accel.copy()
    df_accel_lut['temp'] = np.floor(df_accel_lut['temp'])
    df_accel_lut = pandas.pivot_table(df_accel_lut, values=['x_raw', 'y_raw', 'z_raw'], index='temp', aggfunc=np.mean)
    df_accel_lut = df_accel_lut - np.array(neutral[3:6])

    if plot_results:
        import matplotlib.pyplot as plt

        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(df_gyro['temp'], df_gyro['p_raw'], 'x')
        plt.plot(df_gyro['temp'], df_gyro['q_raw'], 'x')
        plt.plot(df_gyro['temp'], df_gyro['r_raw'], 'x')
        plt.gca().set_prop_cycle(None)
        plt.plot(df_gyro_lut.index.values + 0.5, df_gyro_lut[['p_raw', 'q_raw', 'r_raw']] + np.array([neutral[0:3]]))
        plt.xlabel('Temperature [*C]')
        plt.ylabel('Gyro [raw]')
        plt.subplot(2, 1, 2)
        plt.plot(df_accel['temp'], df_accel['x_raw'], 'x')
        plt.plot(df_accel['temp'], df_accel['y_raw'], 'x')
        plt.plot(df_accel['temp'], df_accel['z_raw'], 'x')
        plt.gca().set_prop_cycle(None)
        plt.plot(df_accel_lut.index.values + 0.5, df_accel_lut[['x_raw', 'y_raw', 'z_raw']] + np.array([neutral[3:6]]))
        plt.xlabel('Temperature [*C]')
        plt.ylabel('Accel [raw]')

        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate IMU temperature LUT from log files')
    parser.add_argument('--ac_id', help='Calibrate only AC_ID')
    parser.add_argument('--plot_results', action='store_true', help='Plot calibration results')
    parser.add_argument('data_files', nargs='+', help='Log .data file', metavar='DATA_FILE')
    args = parser.parse_args()
    kwargs = {k: v for k, v in vars(args).items() if v is not None}

    print(kwargs)
    imu_temp_calib(**kwargs)
