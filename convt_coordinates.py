import json
from pyproj import Proj

# UTM 변환기 설정 (예: 52N 존, WGS84 기준)
utm_proj = Proj(proj="utm", zone=52, ellps="WGS84", south=False)

def convert_coordinates_to_utm(json_file_path):
    # JSON 파일 불러오기
    with open(json_file_path, 'r') as file:
        data = json.load(file)
    
    coordinates = data.get("coordinates", [])
    if not coordinates:
        print("No coordinates found in the file.")
        return

    # 변환된 좌표 저장
    utm_results = []

    print("Converting GPS Coordinates to UTM Coordinates...")
    for point in coordinates:
        lat = point["lat"]
        lng = point["lng"]

        # 위도/경도 → UTM x, y 변환
        x, y = utm_proj(lng, lat)
        utm_results.append({"x": x, "y": y})

        print(f"Lat: {lat}, Lng: {lng} → UTM X: {x:.2f}, UTM Y: {y:.2f}")

    return utm_results

if __name__ == "__main__":
    # JSON 파일 경로 입력
    json_file_path = "get_coordinates"  # 여기에 JSON 파일 경로를 입력하세요.
    utm_coordinates = convert_coordinates_to_utm(json_file_path)

    # 변환된 결과를 출력
    if utm_coordinates:
        print("\nConverted UTM Coordinates:")
        for idx, coord in enumerate(utm_coordinates):
            print(f"Point {idx + 1}: X = {coord['x']:.6f}, Y = {coord['y']:.6f}")
