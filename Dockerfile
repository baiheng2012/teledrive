# ============ 阶段 1：编译并测试 ============
FROM ubuntu:24.04 AS builder
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git ca-certificates libopencv-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . .

# 编译并运行测试：测试失败则整个镜像构建失败
RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DTELEDRIVE_REQUIRE_OPENCV=ON \
    && cmake --build build \
    && ctest --test-dir build --output-on-failure

# ============ 阶段 2：最小运行镜像 ============
FROM ubuntu:24.04 AS runtime
# vehicle_sim 实际链接的 OpenCV 库只有 core、imgproc、imgcodecs（用 ldd 确认过）
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
         libopencv-core406t64 libopencv-imgproc406t64 libopencv-imgcodecs406t64 \
    && rm -rf /var/lib/apt/lists/*

# 只复制车辆端程序；operator_cli 需要图形窗口，在宿主机上运行
COPY --from=builder /src/build/apps/vehicle_sim /usr/local/bin/

# 非 root 用户运行
RUN useradd --create-home --shell /usr/sbin/nologin teledrive
USER teledrive

# 控制端口 9001，视频端口 9002（均为 UDP）
EXPOSE 9001/udp 9002/udp
ENTRYPOINT ["vehicle_sim"]
