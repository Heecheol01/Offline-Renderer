add_executable(render_cpu render_cpu/main.cpp)  # 예시
target_link_libraries(render_cpu PRIVATE or_cpu)

# 이름 충돌을 피하려면 CPU/GPU 앱 이름을 분리하는 걸 추천
# (render, render_cli 같은 동일 이름이면 빌드 산출물이 덮일 수 있음)
