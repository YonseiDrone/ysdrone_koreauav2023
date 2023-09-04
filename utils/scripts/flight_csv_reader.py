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

# WGS84 좌표계를 사용하여 거리와 각도를 계산하는 객체 생성
wgs84_geod = Geodesic.WGS84

# 가장 가까운 점을 찾는 함수 (변환 전)
def find_closest_point(target, data):
    min_dist = float('inf')
    closest_point = None

    for index, row in data.iterrows():
        dist = distance.euclidean((row['Latitude'], row['Longitude']), (target['Latitude'], target['Longitude']))
        if dist < min_dist:
            min_dist = dist
            closest_point = row

    return pd.Series(closest_point)

# 3개의 좌표를 입력
target_coordinates = [
    {'Latitude': 37.5625148, 'Longitude': 126.9366716},
    {'Latitude': 37.5625972, 'Longitude': 126.9367649},
    {'Latitude': 37.5625561, 'Longitude': 126.9365968},
]

# 첫번째 데이터 포인트를 기준점으로 사용
ref_point = data.iloc[0]
ref_lat, ref_lon = ref_point['Latitude'], ref_point['Longitude']

# 위도에 1m 추가
lat_1m = wgs84_geod.Direct(ref_lat, ref_lon, 0, 1)['lat2']  # 0 degree for North
lat_diff_1m = abs(lat_1m - ref_lat)

# 경도에 1m 추가
lon_1m = wgs84_geod.Direct(ref_lat, ref_lon, 90, 1)['lon2']  # 90 degree for East
lon_diff_1m = abs(lon_1m - ref_lon)

print(f"1m difference(WPG84)")
print(f"	in Latitude : {lat_diff_1m:.12f} m")
print(f"	in Longitude: {lon_diff_1m:.12f} m")
print()

# 순서대로 이동할 목적지 좌표 설정 (0 -> 1 -> 2 -> 2 -> 1 -> 0)
travel_sequence = [0, 1, 2, 2, 1, 0]

# 각 목적지에 가장 가까운 점을 찾고 결과를 출력
for i, idx in enumerate(travel_sequence):
    target = target_coordinates[idx]
    closest_point = find_closest_point(target, data)

    lat_diff = abs(target['Latitude'] - closest_point['Latitude']) / lat_diff_1m  # Convert to meters using calculated 1m difference in latitude
    lon_diff = abs(target['Longitude'] - closest_point['Longitude']) / lon_diff_1m  # Convert to meters using calculated 1m difference in longitude

    # 수평 거리 계산 (Euclidean distance)
    horizontal_distance = np.sqrt(lat_diff ** 2 + lon_diff ** 2)
    
    print(f"Step {i + 1} to Target {idx} is closest to Point {closest_point.name}")
    print("[RESULT] Diffrence")
    print(f"	Latitude: {lat_diff:.8f}m, Longitude: {lon_diff:.8f}m")
    print(f"	Total horizontal error: {horizontal_distance:.4f}m")
    print(f"    {abs(closest_point['Altitude'])}")
    print(f"	Vertial error(from 15m): {abs(closest_point['Altitude'] - 15):.8f}m")
    print('-' * 50)


# 경로점별로 색상을 지정
colors = {0: 'red', 1: 'green', 2: 'blue', 3: 'yellow', 4: 'purple'}

# GeoDataFrame으로 변환
gdf = gpd.GeoDataFrame(data, geometry=gpd.points_from_xy(data.Longitude, data.Latitude))

# 지도 시각화
fig, ax = plt.subplots()
gdf.plot(ax=ax, column='경로점', cmap='viridis', legend=True, markersize=10)

# RMSE 계산
mask = data['자동, 수동'] == 1 # "자동,수동"이 1인 행만 선택
rmse = np.sqrt(((data[mask]['Altitude'] - 15) ** 2).mean())

print(f"Altitude 15m와의 RMSE: {rmse}")

plt.show()
