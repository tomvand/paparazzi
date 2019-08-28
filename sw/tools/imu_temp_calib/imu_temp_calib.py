#!/usr/bin/env python3
import argparse
import pandas
import re
import numpy as np


def imu_temp_calib(data_files, ac_id=None, estimate_neutral=False, force_neutral=[], **kwargs):
    if kwargs['plot_results']:
        import matplotlib.pyplot as plt

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

    if kwargs['plot_results']:
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(df_gyro['temp'], df_gyro['p_raw'], 'x')
        plt.plot(df_gyro['temp'], df_gyro['q_raw'], 'x')
        plt.plot(df_gyro['temp'], df_gyro['r_raw'], 'x')
        plt.xlabel('Temperature [*C]')
        plt.ylabel('Gyro [raw]')
        plt.subplot(2, 1, 2)
        plt.plot(df_accel['temp'], df_accel['x_raw'], 'x')
        plt.plot(df_accel['temp'], df_accel['y_raw'], 'x')
        plt.plot(df_accel['temp'], df_accel['z_raw'], 'x')
        plt.xlabel('Temperature [*C]')
        plt.ylabel('Accel [raw]')

    # Find and correct neutral values
    neutral = []
    if force_neutral:
        print('Using provided neutral values:')
        print(force_neutral)
        neutral = force_neutral
    elif estimate_neutral or \
            len(set(df_gyro['p_neutral'])) != 1 or \
            len(set(df_gyro['q_neutral'])) != 1 or \
            len(set(df_gyro['r_neutral'])) != 1 or \
            len(set(df_accel['x_neutral'])) != 1 or \
            len(set(df_accel['y_neutral'])) != 1 or \
            len(set(df_accel['z_neutral'])) != 1:
        print('Warning: neutral values vary within log files!')
        print('Neutral values will be estimated instead...')
        neutral = [
            np.mean(df_gyro['p_raw']),
            np.mean(df_gyro['q_raw']),
            np.mean(df_gyro['r_raw']),
            np.mean(df_accel['x_raw']),
            np.mean(df_accel['y_raw']),
            np.mean(df_accel['z_raw'])
        ]
    else:
        print('Using neutral values from log file.')
        neutral = [
            df_gyro['p_neutral'][0],
            df_gyro['q_neutral'][0],
            df_gyro['r_neutral'][0],
            df_accel['x_neutral'][0],
            df_accel['y_neutral'][0],
            df_accel['z_neutral'][0]
        ]

    # Estimate gravity
    if np.abs(np.mean(df_accel['z_raw'])) > 50 * np.abs(np.mean(df_accel['x_raw'])) and \
            np.abs(np.mean(df_accel['z_raw'])) > 50 * np.abs(np.mean(df_accel['y_raw'])):
        print('Assume gravity along z axis.')
        gravity = np.array([0., 0., np.mean(df_accel['z_raw'])]) - np.array(neutral[3:6])
    else:
        print('Error: gravity not along z axis!')
        print('Will not ignore gravity in bias estimation...')
        gravity = np.array([0., 0., 0.])

    # Generate LUT
    df_gyro['temp'] = np.floor(df_gyro['temp'])
    df_gyro_bias = pandas.pivot_table(df_gyro, values=['p_raw', 'q_raw', 'r_raw'], index='temp', aggfunc=np.mean)
    df_gyro_bias = df_gyro_bias - np.array(neutral[0:3])
    df_accel['temp'] = np.floor(df_accel['temp'])
    df_accel_bias = pandas.pivot_table(df_accel, values=['x_raw', 'y_raw', 'z_raw'], index='temp', aggfunc=np.mean)
    df_accel_bias = df_accel_bias - np.array(neutral[3:6]) - gravity

    if kwargs['plot_results']:
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate IMU temperature LUT from log files')
    parser.add_argument('--ac_id', help='Calibrate only AC_ID')
    parser.add_argument('--estimate_neutral', action='store_true', help='Estimate neutral values from log data')
    parser.add_argument('--force_neutral', nargs=6, type=int, help='Use the given neutral values',
                        metavar=('P', 'Q', 'R', 'X', 'Y', 'Z'))
    parser.add_argument('--plot_results', action='store_true', help='Plot calibration results')
    parser.add_argument('data_files', nargs='+', help='Log .data file', metavar='DATA_FILE')
    args = parser.parse_args()
    kwargs = {k: v for k, v in vars(args).items() if v is not None}

    print(kwargs)
    imu_temp_calib(**kwargs)
