import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial  import distance
from pyproj import Geod
from geographiclib.geodesic import Geodesic
import argparse

# Argparse 설정
parser = argparse.ArgumentParser(description="Find closest points based on target coordinates.")
parser.add_argument("file_name", help="Name of the CSV file containing drone flight data.",default='yonsei_drone_flight_log_2023-08-31 18:20:34.611985.csv', nargs='?')
args = parser.parse_args()

# 파일 이름을 arg에서 가져오기
file_name = args.file_name

# CSV 파일을 읽어서 DataFrame으로 저장
data = pd.read_csv(file_name)

# RMSE 계산
mask = data['자동, 수동'] == 1 # "자동,수동"이 1인 행만 선택
mask_2 = data['경로점'] != 4
data_auto = data[mask]['Altitude']
data_wpt = data_auto[mask_2]
rmse = np.sqrt(((data_wpt - 15) ** 2).mean())

print(f"Altitude 15m와의 RMSE: {rmse}")

plt.show()
