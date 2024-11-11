import pandas as pd

# 원본 CSV 파일 이름
input_filename = 'gps1.csv'
# 결과를 저장할 새로운 CSV 파일 이름
output_filename = 'gps1_n.csv'

# 원본 CSV 파일 읽기
df = pd.read_csv(input_filename, encoding='utf-8')

# 2줄씩 건너뛰기 위한 조건을 만족하는 행만 필터링
# 여기서는 인덱스를 이용해서 3의 배수인 행만 선택합니다. (인덱스 0부터 시작하므로 이렇게 하면 2줄 건너뛰고 하나를 선택하는 효과가 있습니다.)
filtered_df = df.iloc[::3]

# 결과 데이터를 새로운 CSV 파일에 쓰기
filtered_df.to_csv(output_filename, index=False, encoding='utf-8')

print(f'{output_filename} 파일에 2줄씩 건너뛴 데이터를 저장했습니다.')
