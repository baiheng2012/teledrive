# 基础镜像：Ubuntu 24.04
FROM ubuntu:24.04

# 安装编译所需工具
#   --no-install-recommends : 不装推荐包，镜像更小
#   ca-certificates         : 让 FetchContent 能通过 https 下载 GoogleTest
#   最后 rm -rf 清理 apt 缓存，减小镜像体积
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# 容器内的工作目录
WORKDIR /src

# 把当前目录（项目源码）复制进容器
COPY . .

# 构建并运行测试：如果测试失败，docker build 也会失败
RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build \
    && ctest --test-dir build --output-on-failure
