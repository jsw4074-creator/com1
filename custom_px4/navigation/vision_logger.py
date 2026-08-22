#!/usr/bin/env python3

import csv
from datetime import datetime


class VisionLogger:
    def __init__(self, save_path=None):
        if save_path is None:
            now = datetime.now().strftime('%Y%m%d_%H%M%S')
            save_path = f'vision_log_{now}.csv'
        self.save_path = save_path
        self.rows = []

    def log(self, timestamp_us, angle, altitude, score,
            top_val, mid_val, bot_val):
        self.rows.append([
            timestamp_us,
            angle,
            altitude,
            score,
            top_val,
            mid_val,
            bot_val
        ])

    def save(self):
        with open(self.save_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp_us',
                'angle_deg',
                'altitude_m',
                'score',
                'top_val',
                'mid_val',
                'bot_val'
            ])
            writer.writerows(self.rows)
        print(f'\n저장 완료: {self.save_path} ({len(self.rows)}행)')
